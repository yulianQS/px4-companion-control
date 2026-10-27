#!/usr/bin/env python3
"""局域网真实飞控控制台。启动：python3 web_control.py。"""
import json
import os
import signal
import socket
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from contextlib import nullcontext
from .config import ROOT, RUNTIME

sys.dont_write_bytecode = True
from .cpp_bridge import ControlError, FlightConsole, clean
from .flight_analysis import FlightArchive, analyze_csv
from .vision_snapshots import VisionSnapshots, VisionPreviewConfig
from .vision_control_config import VisionControlConfig
from .mission_catalog import CATALOG, CATEGORIES
from .mission_store import MissionStore, validate as validate_mission, execution_issues
from .mission_client import MissionClient

HOST, PORT = RUNTIME['web']['host'], RUNTIME['web']['port']
WEB_DIRECTORY = ROOT / 'web'
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/style.css': ('style.css', 'text/css; charset=utf-8'),
          '/app.js': ('app.js', 'text/javascript; charset=utf-8')}
ASSETS['/motion_workspace.js'] = ('motion_workspace.js', 'text/javascript; charset=utf-8')
ASSETS['/vision_ros.js'] = ('vision_ros.js', 'text/javascript; charset=utf-8')
ASSETS['/mission.css'] = ('mission.css', 'text/css; charset=utf-8')
ASSETS['/mission_model.js'] = ('mission_model.js', 'text/javascript; charset=utf-8')
ASSETS['/mission_ui.js'] = ('mission_ui.js', 'text/javascript; charset=utf-8')
ASSETS['/cpp_controls.js'] = ('cpp_controls.js', 'text/javascript; charset=utf-8')
ASSETS['/joystick.js'] = ('joystick.js', 'text/javascript; charset=utf-8')
ASSETS['/map3d.js'] = ('map3d.js', 'text/javascript; charset=utf-8')
ASSETS['/trajectory-view'] = ('trajectory.html', 'text/html; charset=utf-8')
ASSETS['/trajectory.css'] = ('trajectory.css', 'text/css; charset=utf-8')
ASSETS['/trajectory_plot.js'] = ('trajectory_plot.js', 'text/javascript; charset=utf-8')
ASSETS['/vendor/plotly-strict-4.1.1.min.js'] = ('vendor/plotly-strict-4.1.1.min.js', 'text/javascript; charset=utf-8')


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, body, content='application/json; charset=utf-8', plotly_page=False):
        self.send_response(code)
        self.send_header('Content-Type', content)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        # Plotly动态样式仅限独立只读图表页；控制台沿用原策略与接口。
        policy = ("default-src 'self'; img-src 'self' data: blob:; style-src 'self' 'unsafe-inline'; "
                  "script-src 'self'; connect-src 'self'; frame-ancestors 'self'; base-uri 'none'; form-action 'none'"
                  if plotly_page else
                  "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        self.send_header('Content-Security-Policy', policy)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ASSETS:
            filename, content = ASSETS[path]
            self.reply(200, (WEB_DIRECTORY / filename).read_bytes(), content,
                       plotly_page=path == '/trajectory-view')
        elif path == '/api/mission/catalog':
            self.json_reply(200, dict(ok=True, categories=CATEGORIES, blocks=CATALOG))
        elif path == '/api/mission/status':
            try:
                self.json_reply(200, dict(ok=True, mission=self.server.mission.status()))
            except ControlError as error:
                self.json_reply(error.code, dict(ok=False, message=str(error)))
        elif path == '/api/mission/list':
            self.json_reply(200, dict(ok=True, missions=self.server.mission_store.list()))
        elif path == '/api/mission/load':
            try:
                id_ = parse_qs(urlsplit(self.path).query).get('id', [''])[0]
                self.json_reply(200, dict(ok=True, document=self.server.mission_store.load(id_)))
            except (OSError, ValueError):
                self.json_reply(404, dict(ok=False, message='任务不存在或格式无效'))
        elif path == '/api/vision/control-config':
            try:
                self.json_reply(200, self.server.vision_control_config.load())
            except (OSError, ValueError) as error:
                self.json_reply(503, {'ok': False, 'message': '读取视觉默认参数失败：' + str(error)})
        elif path == '/api/vision/status':
            self.json_reply(200, self.server.vision.status())
        elif path == '/api/vision/stream.mjpg':
            self.vision_stream()
        elif path == '/api/vision/frame.jpg':
            try:
                self.reply(200, self.server.vision.frame(), 'image/jpeg')
            except (OSError, ValueError):
                self.reply(404, b'{"error":"camera_unavailable_or_stale"}')
        elif path == '/api/status':
            status = self.server.console.status()
            vision = getattr(self.server, 'vision', None)
            status['data']['vision'] = vision.status() if vision else VisionSnapshots.unavailable('视觉未启动')
            recorder = getattr(self.server, 'recorder', None)
            status['data'].setdefault('trajectory', recorder.status() if recorder else {})
            self.json_reply(200, status)
        elif path in ('/api/trajectory/latest', '/api/trajectory/latest.csv'):
            try:
                if path.endswith('.csv'):
                    # 名称仅来自记录器，不接受客户端路径。
                    self.reply(200, self.server.recorder.latest_file().read_bytes(), 'text/csv; charset=utf-8')
                else:
                    self.json_reply(200, self.server.recorder.latest_trajectory())
            except FileNotFoundError as error:
                self.json_reply(404, {'message': str(error)})
            except (OSError, ValueError):
                self.json_reply(500, {'message': '无法读取轨迹记录，请检查开发板文件'})
        else:
            self.reply(404, b'{"error":"not_found"}')

    def vision_stream(self):
        """Bounded latest-frame MJPEG; slow clients drop frames rather than build a queue."""
        self.connection.settimeout(2)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=visionframe')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        previous = None
        last_sent = time.monotonic()
        try:
            while time.monotonic() - last_sent < 1.5:
                try:
                    frame_path = self.server.vision.directory / 'latest.jpg'
                    identity = frame_path.stat().st_mtime_ns
                    if identity != previous:
                        body = self.server.vision.frame()
                    else:
                        body = None
                except (OSError, ValueError):
                    body = None  # Brief source gaps do not immediately tear down TCP.
                if body is not None:
                    self.wfile.write(b'--visionframe\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                     str(len(body)).encode('ascii') + b'\r\n\r\n' + body + b'\r\n')
                    self.wfile.flush()
                    previous = identity
                    last_sent = time.monotonic()
                time.sleep(1 / 30)
        except OSError:
            pass  # Slow/disconnected client; no queued history and no flight-side effects.
        self.close_connection = True

    def do_POST(self):
        self.connection.settimeout(5)
        try:
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                raise ControlError('不接受跨站控制请求', 403)
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                raise ControlError('需要 application/json', 415)
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= (16 * 1024 * 1024 if urlsplit(self.path).path == '/api/trajectory/analyze' else 262144):
                raise ControlError('请求大小无效', 413)
            if self.headers.get('X-Control-Request') != '1':
                raise ControlError('请使用网页实机控制按钮', 403)
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ControlError('请求必须是对象', 400)
            client = self.headers.get('X-Client-ID', '')
            path = urlsplit(self.path).path
            mission = getattr(self.server, 'mission', None)
            guarded = path in ('/api/mission/start', '/api/control/action', '/api/control/stick', '/api/control/claim', '/api/service/connect', '/api/service/disconnect', '/api/vision/preview')
            with mission.command_gate if mission and guarded else nullcontext():
                if mission and mission.active and path in ('/api/control/action', '/api/control/stick', '/api/control/claim', '/api/service/connect', '/api/service/disconnect', '/api/vision/preview'):
                    raise ControlError('比赛任务正在执行，请先在比赛编排中停止；降落请使用“停止并降落”', 409)
                if path == '/api/mission/validate':
                    document = validate_mission(body)
                    result = dict(ok=True, document=document, issues=execution_issues(document))
                elif path == '/api/mission/save':
                    document = self.server.mission_store.save(body)
                    result = dict(ok=True, document=document, message='比赛任务已保存到工程 missions 目录')
                elif path == '/api/mission/start':
                    result = self.server.mission.start(body, client)
                elif path == '/api/mission/stop':
                    result = self.server.mission.stop(body.get('mode', 'hold'), client=client)
                elif path == '/api/mission/emergency-stop':
                    result = self.server.mission.emergency_stop(client)
                elif path == '/api/vision/control-config':
                    result = self.server.vision_control_config.save(body)
                elif path == '/api/vision/preview':
                    motion = self.server.console.status().get('data', {}).get('motion', {})
                    if motion.get('state') == 'running' and motion.get('params', {}).get('kind') == 'vision_follow':
                        raise ControlError('先停止实际视觉控制，再修改参数', 409)
                    result = self.server.vision_config.apply(body)
                elif path == '/api/service/connect':
                    result = self.server.console.client.configure(body.get('host'), body.get('port'), True)
                elif path == '/api/service/disconnect':
                    connection = self.server.console.client
                    result = connection.configure(connection.host, connection.port, False)
                elif path == '/api/trajectory/analyze':
                    result = analyze_csv(body.get('csv', ''), body.get('filename', 'import.csv'))
                elif path == '/api/control/claim':
                    result = self.server.console.claim(client)
                elif path == '/api/control/renew':
                    result = self.server.console.renew(client)
                elif path == '/api/control/action':
                    result = self.server.console.perform(client, body)
                elif path == '/api/control/stick':
                    result = self.server.console.stick_input(client, body)
                else:
                    raise ControlError('接口不存在', 404)
            self.json_reply(200, result)
        except ControlError as error:
            self.json_reply(error.code, {'ok': False, 'message': str(error)})
        except (ValueError, UnicodeError, OSError) as error:
            if urlsplit(self.path).path.startswith('/api/mission/'):
                self.json_reply(400, {'ok': False, 'message': '比赛任务请求无效：' + str(error)})
                return
            self.json_reply(400, {'ok': False, 'message': '请求无效或连接中断'})
        except Exception as error:
            # 未知错误不得伪装成功，也不自动重发可能已执行的命令。
            self.json_reply(500, {'ok': False, 'message': '操作结果未知：' + str(error)})

    def json_reply(self, code, data):
        self.reply(code, json.dumps(clean(data), ensure_ascii=False, allow_nan=False).encode())


def main():
    recorder = FlightArchive()
    console = FlightConsole(host=os.environ.get('DRONE_SERVICE_HOST', RUNTIME['control']['host']),
                            port=int(os.environ.get('DRONE_SERVICE_PORT', RUNTIME['control']['port'])))
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    server.console = console
    server.recorder = recorder
    server.vision = VisionSnapshots(os.environ.get('DRONE_VISION_SNAPSHOT_DIR', RUNTIME['vision_snapshot_dir']))
    server.vision_config = VisionPreviewConfig(server.vision)
    server.vision_control_config = VisionControlConfig()
    server.mission_store = MissionStore()
    server.mission = MissionClient(console)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print('Python网页上位机 → TCP/JSON → C++控制服务；Python不访问飞控串口。', flush=True)
    print('网页点击“启用实机控制”即可操作，无需口令。', flush=True)
    addresses = set(socket.gethostbyname_ex(socket.gethostname())[2])
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(('192.168.137.1', 80))
            addresses.add(probe.getsockname()[0])
    except OSError:
        pass
    for address in sorted(addresses):
        if not address.startswith('127.'):
            print('浏览器访问：http://%s:%d/' % (address, PORT), flush=True)
    try:
        console.start()
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        console.stop()
        server.server_close()
        print('控制台已关闭。', flush=True)


if __name__ == '__main__':
    main()
