#!/usr/bin/env python3
"""局域网真实飞控控制台。启动：python3 web_control.py。"""
import json
import os
import signal
import socket
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
import settings

sys.dont_write_bytecode = True
if __name__ == '__main__' and sys.prefix != settings.PYTHON_ENV:
    os.execv(settings.PYTHON_ENV + '/bin/python',
             [settings.PYTHON_ENV + '/bin/python', '-B', '-u', os.path.abspath(__file__)])

from web_flight import ControlError, FlightConsole, clean

HOST, PORT = '0.0.0.0', 8081
WEB_DIRECTORY = Path(__file__).resolve().parent / 'web'
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/style.css': ('style.css', 'text/css; charset=utf-8'),
          '/app.js': ('app.js', 'text/javascript; charset=utf-8')}
ASSETS['/joystick.js'] = ('joystick.js', 'text/javascript; charset=utf-8')


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, body, content='application/json; charset=utf-8'):
        self.send_response(code)
        self.send_header('Content-Type', content)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ASSETS:
            filename, content = ASSETS[path]
            self.reply(200, (WEB_DIRECTORY / filename).read_bytes(), content)
        elif path == '/api/status':
            self.json_reply(200, self.server.console.status())
        else:
            self.reply(404, b'{"error":"not_found"}')

    def do_POST(self):
        self.connection.settimeout(5)
        try:
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                raise ControlError('不接受跨站控制请求', 403)
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                raise ControlError('需要 application/json', 415)
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ControlError('请求大小无效', 413)
            if self.headers.get('X-Control-Request') != '1':
                raise ControlError('请使用网页实机控制按钮', 403)
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ControlError('请求必须是对象', 400)
            client = self.headers.get('X-Client-ID', '')
            path = urlsplit(self.path).path
            if path == '/api/control/claim':
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
        except (ValueError, UnicodeError, OSError):
            self.json_reply(400, {'ok': False, 'message': '请求无效或连接中断'})
        except Exception as error:
            # 未知错误不得伪装成功，也不自动重发可能已执行的命令。
            self.json_reply(500, {'ok': False, 'message': '操作结果未知：' + str(error)})

    def json_reply(self, code, data):
        self.reply(code, json.dumps(clean(data), ensure_ascii=False, allow_nan=False).encode())


def main():
    # test.py/operator_cli.py 与本入口互斥，启动前拒绝占用中的 UART。
    import subprocess
    busy = subprocess.run(['fuser', settings.UART_DEVICE], capture_output=True, text=True)
    if busy.returncode == 0:
        raise RuntimeError('UART7 正被其他程序占用，请先退出 test.py/operator_cli.py')
    console = FlightConsole()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    server.console = console
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print('真实MAVLink控制台；支持未解锁地面摇杆调试，解锁/上锁由用户操作。', flush=True)
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
