"""Focused PTY-only framing, motor-cancel and interrupted-log regression."""
from verify_cpp_service import FakePX4, wait, ServiceClient
from web.server.flight_analysis import FlightArchive
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

binary = Path(sys.argv[1] if len(sys.argv) > 1 else 'build/drone_service').resolve()
fake = FakePX4()
client = None
process = None
with tempfile.TemporaryDirectory(prefix='cpp-protocol-test-') as temp:
    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        fake.thread.start()
        process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port), '--logs', temp], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        client = ServiceClient(port=port)
        client.start()
        wait(lambda: client.status()['data']['px4'].get('bidirectional'), 8)
        with socket.create_connection(('127.0.0.1', port)) as sock:
            sock.settimeout(3)
            # Two frames coalesced, then fragmented across header/body boundaries.
            blob = b''
            for identifier in ('fragment-a', 'fragment-b'):
                body = json.dumps(dict(id=identifier, type='command', command='GET_STATUS', params={})).encode()
                blob += struct.pack('!I', len(body)) + body
            for start in range(0, len(blob), 3):
                sock.sendall(blob[start:start + 3])
            buffer, ids = bytearray(), set()
            while len(ids) < 2:
                buffer.extend(sock.recv(65536))
                while len(buffer) >= 4:
                    size = struct.unpack_from('!I', buffer)[0]
                    if len(buffer) < size + 4:
                        break
                    reply = json.loads(buffer[4:size + 4])
                    del buffer[:size + 4]
                    if reply.get('type') == 'response':
                        ids.add(reply['id'])
            assert ids == {'fragment-a', 'fragment-b'}
        with socket.create_connection(('127.0.0.1', port)) as sock:
            sock.sendall(struct.pack('!I', 1048577))  # Existing service limit is 1 MiB.
            sock.settimeout(2)
            # A status broadcast can already be queued before the invalid header.
            # Drain it and require EOF; a connection left open still times out.
            while sock.recv(65536):
                pass
        owner = 'protocol-test-owner'
        assert client.request('CLAIM', {}, owner)['ok']
        result = []
        thread = threading.Thread(target=lambda: result.append(client.request('MOTOR_TEST', {'props_removed': True}, owner, timeout=15)))
        thread.start()
        wait(lambda: len([m for m in fake.commands if m.command == 310]) >= 4)
        begun = time.monotonic()
        assert client.request('STOP_MOTOR_TEST', {}, owner)['stage'] == 'accepted'
        thread.join(3)
        assert not thread.is_alive() and time.monotonic() - begun < 3, 'Stop must bypass slow worker'
        motors = [m for m in fake.commands if m.command == 310]
        assert all(any(m.param5 == channel and math.isnan(m.param1) for m in motors) for channel in (1, 2, 3, 4))
        assert all(m.param2 <= 1 for m in motors), 'Per-output timeout must stay at most one second'
        assert client.status()['data']['px4']['armed'] is False
        # A fake armed session interrupted by service shutdown is never a complete flight.
        fake.armed = True
        wait(lambda: client.status()['data']['trajectory']['recording'])
        time.sleep(.2)
        client.stop()
        process.terminate()
        process.wait(5)
        assert list(Path(temp).glob('*.interrupted.csv'))
        try:
            FlightArchive(temp).latest_file()
        except FileNotFoundError:
            pass
        else:
            raise AssertionError('Interrupted record must not be latest completed flight')
        print('PASS: TCP fragmented/coalesced/oversize frames; PTY motor-cancel path; interrupted CSV excluded')
    finally:
        if client:
            client.stop()
        if process and process.poll() is None:
            process.terminate()
            process.wait(8)
        if process and process.returncode != 0:
            print(process.communicate())
        fake.close()
