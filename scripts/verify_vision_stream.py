import json,threading,time,tempfile,urllib.request
from pathlib import Path
from http.server import ThreadingHTTPServer
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.http import Handler
from web.server.vision_snapshots import VisionSnapshots
with tempfile.TemporaryDirectory() as folder:
    root=Path(folder)
    def status(ready):
        tmp=root/'status.tmp'
        tmp.write_text(json.dumps(dict(source='ros2_cpp',written_monotonic=time.monotonic(),camera_ready=ready)))
        tmp.replace(root/'status.json')
    def frame(n):
        tmp=root/'frame.tmp';tmp.write_bytes(b'\xff\xd8'+bytes([n])+b'\xff\xd9');tmp.replace(root/'latest.jpg')
    status(True);frame(0)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    server.daemon_threads=True;server.vision=VisionSnapshots(root)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        import json
        with urllib.request.urlopen('http://127.0.0.1:%d/api/vision/status'%server.server_port,timeout=2) as response:
            assert json.load(response)['camera_ready'] is True
        with urllib.request.urlopen('http://127.0.0.1:%d/api/vision/stream.mjpg'%server.server_port,timeout=2) as r:
            assert r.headers['Content-Type'].startswith('multipart/')
            for n in range(4):
                if n:status(True);frame(n)
                assert r.readline().strip()==b'--visionframe'
                assert r.readline().strip()==b'Content-Type: image/jpeg'
                length=int(r.readline().split(b':')[1]);assert r.readline()==b'\r\n'
                data=r.read(length);assert data==b'\xff\xd8'+bytes([n])+b'\xff\xd9'
                assert r.readline()==b'\r\n'
            status(False)
            assert r.readline()==b'', 'stale/disabled camera must terminate stream'
        print('PASS: MJPEG framing, latest-frame updates, camera loss closes stream')
    finally:
        server.shutdown();server.server_close();thread.join()
