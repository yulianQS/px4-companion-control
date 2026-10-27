"""Plotly静态资源和隔离页面策略检查；只开临时HTTP端口，不访问UART。"""
from pathlib import Path
import sys
import threading
from urllib.request import urlopen
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.http import Handler, ThreadingHTTPServer

server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
thread = threading.Thread(target=server.serve_forever, daemon=True)
thread.start()
try:
    base = 'http://127.0.0.1:' + str(server.server_port)
    for route in ('/', '/map3d.js', '/trajectory-view', '/trajectory.css', '/trajectory_plot.js', '/vendor/plotly-strict-4.1.1.min.js'):
        with urlopen(base + route, timeout=5) as response:
            assert response.status == 200
            body = response.read()
            assert body
            policy = response.headers['Content-Security-Policy']
            assert "script-src 'self'" in policy and 'unsafe-eval' not in policy
            if route == '/':
                assert "style-src 'self';" in policy and "frame-ancestors 'none'" in policy
                assert b'trajectory-frame' in body and b'map3d-scene' not in body
            elif route == '/trajectory-view':
                assert "frame-ancestors 'self'" in policy
                assert b'/vendor/plotly-strict-4.1.1.min.js' in body
            elif route.startswith('/vendor/'):
                assert len(body) > 1000000 and b'plotly.js (strict' in body[:300]
    print('PASS: Plotly本地资源、只读iframe、控制台原CSP与脚本策略保持')
finally:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)
