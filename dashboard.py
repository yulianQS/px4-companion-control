"""只读网页：摄像头与飞控状态；没有网页电机/解锁按钮。"""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import settings

PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PX4 视觉伴随计算机</title>
<style>body{background:#111827;color:#eee;font:16px system-ui;margin:auto;max-width:1000px;padding:20px}img{width:100%}pre{white-space:pre-wrap;background:#1f2937;padding:16px}</style>
<h2>PX4 · AprilTag 视觉链路</h2><p>当前只读与控制预览，不解锁、不发送飞行设定值。</p>
<img src="/stream.mjpg"><pre id="status">连接中…</pre>
<script>async function update(){try{let s=await(await fetch('/status.json',{cache:'no-store'})).json();
document.getElementById('status').textContent=JSON.stringify(s,null,2)}catch(e){document.getElementById('status').textContent='连接中断'}setTimeout(update,1000)}update()</script></html>'''.encode()


def create_server(store, stop_event, snapshot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.connection.settimeout(3)
                sequence = -1
                try:
                    while not stop_event.is_set():
                        with store.condition:
                            store.condition.wait_for(lambda: store.sequence != sequence or stop_event.is_set(), 2)
                            if stop_event.is_set():
                                break
                            sequence, jpeg = store.sequence, store.jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                return
            if path == "/":
                body, content = PAGE, "text/html; charset=utf-8"
            elif path == "/status.json":
                body, content = json.dumps(snapshot(), ensure_ascii=False).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer((settings.WEB_HOST, settings.WEB_PORT), Handler)
    server.daemon_threads = True
    return server

