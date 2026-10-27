"""Accepted autonomous tasks outlive GUI leases. Uses ONLY fresh PTY PX4 peers."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from verify_cpp_service import FakePX4, ServiceClient, wait


def check(binary, kind):
    fake = FakePX4()
    assert fake.device.startswith('/dev/pts/')
    client = process = None
    with tempfile.TemporaryDirectory(prefix='onboard-disconnect-') as temp:
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            fake.thread.start()
            process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port),
                                        '--logs', temp], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            client = ServiceClient(port=port)
            client.start()
            owner = 'disconnect-regression-' + kind
            def state(): return client.status()['data']
            def command(name, params=None): return client.request(name, params or {}, owner)
            wait(lambda: state()['px4'].get('bidirectional') and
                 'takeoff_speed_m_s' in state()['px4'], 8)
            assert command('CLAIM')['ok']
            lease_started = time.monotonic()
            assert command('SET_MODE', {'mode': 'OFFBOARD'})['confirmed']
            assert command('ARM')['confirmed']
            fake.follow_setpoints = False
            if kind == 'takeoff':
                result = command('TAKEOFF', {'height': .5, 'timeout': 40})
            else:
                fake.landed = 2
                wait(lambda: state()['px4'].get('landed_state') == 2)
                if kind == 'velocity':
                    # Start a bounded 10-second velocity task just before lease expiry.
                    time.sleep(max(0, lease_started+16-time.monotonic()))
                    result = command('SET_VELOCITY', dict(frame='LOCAL_NED', vx=.1, duration=10))
                else:
                    result = command('MOVE', dict(kind='move', direction='forward', distance=.5, timeout=40))
            assert result.get('stage') == 'running', result
            client.stop()  # No API socket, browser heartbeat or RENEW during the interval.
            time.sleep(max(0, lease_started+22-time.monotonic()))
            client = ServiceClient(port=port)
            client.start()
            wait(lambda: state()['px4'].get('bidirectional'))
            data = state()
            assert data['control']['lease_active'] is False, data['control']
            assert data['motion']['state'] == 'running', data['motion']
            recent = [stamp for stamp, _ in fake.setpoints if stamp > time.monotonic()-1]
            assert len(recent) >= 12, recent
            # Disconnection must not synthesize LAND or revoke manual arming.
            assert fake.armed and fake.mode == (6, 0), (fake.armed, fake.mode)
            assert command('MOVE', dict(kind='move', direction='forward', distance=.2))['stage'] == 'failed'
            fake.follow_setpoints = True
            wait(lambda: state()['motion']['state'] == 'succeeded', 8)
            assert command('CLAIM')['ok']
            assert command('HOLD')['confirmed']
            print('PASS', kind, ': API absent, lease expired, task and 20Hz stream continue; explicit HOLD works', flush=True)
        finally:
            if client:
                client.stop()
            if process:
                process.terminate()
                try:
                    process.communicate(timeout=6)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
            fake.close()
            if hasattr(fake, 'error'):
                raise fake.error


if __name__ == '__main__':
    binary = Path(sys.argv[1]).resolve()
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(lambda kind: check(binary, kind), ('takeoff', 'move', 'velocity')))
