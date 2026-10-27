"""Linux: durable console capture plus SIGKILL/restart recovery using ONLY a PTY."""
import csv
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import io
from unittest.mock import patch
from verify_cpp_service import FakePX4, ServiceClient, wait
import log_storage

ROOT = Path(__file__).resolve().parents[1]
binary = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='log-storage-test-') as temp:
    root = Path(temp)
    log = root/'20261023T000000Z-console.log'
    capture = subprocess.Popen([sys.executable, '-B', str(ROOT/'scripts/runtime_logging.py'),
        '--log', str(log), '--', sys.executable, '-u', '-c',
        'import time; print("checkpoint-visible"); time.sleep(2)'])
    wait(lambda: log.exists() and b'checkpoint-visible' in log.read_bytes(), 4)
    capture.wait(5)
    metadata = json.loads(log.with_suffix('.json').read_text())
    assert metadata['state'] == 'completed' and metadata['boot_id'] and metadata['ended_utc']
    old_root = log_storage.ROOT
    log_storage.ROOT = root
    archived = root/'logs/runtime/20200101T000000Z'
    archived.mkdir(parents=True)
    log_storage.write_json(archived/'old-control.json', dict(state='running', command=['fake'],
                           boot_id='previous-boot', logger_pid=999999))
    log_storage.recover_sessions()
    assert json.loads((archived/'old-control.json').read_text())['state'] == 'interrupted'
    import start_runtime
    with patch.object(start_runtime, 'LOGS', root/'logs/runtime'), \
         patch.object(start_runtime, 'listening', return_value=True), \
         patch.object(start_runtime, 'processes', return_value=[999999]), \
         patch.object(start_runtime.urllib.request, 'urlopen', side_effect=lambda *a,**k: io.BytesIO(
             b'{"source":"ros2_cpp","camera_ready":true}')):
        start_runtime.main()  # Existing services are preserved; no device is opened.
    assert json.loads((start_runtime.SESSION/'session.json').read_text())['state']=='ready'
    log_storage.ROOT = old_root
    fake = FakePX4()
    client = process = None
    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0)); port = probe.getsockname()[1]
        fake.thread.start()
        def start():
            child = subprocess.Popen([str(binary),'--device',fake.device,'--port',str(port),
                                      '--logs',str(root/'flights')],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            api = ServiceClient(port=port);api.start()
            wait(lambda:api.status()['data']['px4'].get('bidirectional'),8)
            return child,api
        process,client = start()
        fake.armed = True
        wait(lambda:client.status()['data']['trajectory'].get('points',0)>10,5)
        time.sleep(1.2)
        partial = next((root/'flights').glob('*.partial.csv'))
        with partial.open() as file:
            rows = list(csv.DictReader(file))
        assert rows and rows[-1]['boot_id'] and rows[-1]['monotonic_s']
        process.kill();process.wait(5);client.stop()
        assert partial.exists()
        fake.close()  # TIOCEXCL survives SIGKILL while the fixture holds its slave FD.
        fake = FakePX4();fake.thread.start()
        process,client = start()
        assert not partial.exists()
        interrupted = partial.with_name(partial.name.replace('.partial.csv','.interrupted.csv'))
        assert interrupted.exists()
        fake.armed = False
        wait(lambda:not client.status()['data']['trajectory']['recording'],5)
        assert interrupted.read_text().startswith('timestamp,')
        print('PASS: durable process metadata, CSV checkpoint, SIGKILL recovery and additive CSV fields')
    finally:
        if client: client.stop()
        if process and process.poll() is None:
            process.terminate();process.wait(8)
        fake.close()
