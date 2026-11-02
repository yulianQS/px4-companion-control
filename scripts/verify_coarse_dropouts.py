"""Test coarse visual handoff with simulated snapshots and a PTY-only PX4."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

from verify_cpp_service import FakePX4, wait
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.cpp_bridge import FlightConsole
from web.server.mission_client import MissionClient


def run():
    binary = Path(sys.argv[1]).resolve()
    fake = FakePX4()
    assert fake.device.startswith('/dev/pts/')
    done = threading.Event()
    mode = ['frozen']
    process = console = None
    with tempfile.TemporaryDirectory(prefix='coarse-pty-') as temp:
        root = Path(temp)
        (root / 'vision').mkdir()
        (root / 'config').mkdir()
        (root / 'config/mission_blocks.json').write_bytes(
            (Path(__file__).resolve().parents[1] / 'config/mission_blocks.json').read_bytes())
        def publisher():
            count = 0
            stamp = time.monotonic()
            while not done.is_set():
                if mode[0] != 'frozen':
                    stamp = time.monotonic()
                sample = dict(source='ros2_cpp', written_monotonic=time.monotonic(),
                              observation_monotonic=stamp, camera_ready=True,
                              target_found=(mode[0] == 'frozen' or count % 8 < 2),
                              tag_id=0, center=[359.5, 339.5], width=640, height=480,
                              measurement_age_s=.01, pose_valid=True, calibrated=True,
                              tag_size_m=.12, distance_m=2., translation_m=[.3, -.1, 2.])
                pending = root / 'vision/status.tmp'
                pending.write_text(json.dumps(sample))
                pending.replace(root / 'vision/status.json')
                done.wait(.03)
                count += 1
        thread = threading.Thread(target=publisher, daemon=True)
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
            thread.start(); fake.thread.start()
            process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port),
                                        '--logs', str(root / 'flights')], stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, env=dict(os.environ, DRONE_PROJECT_ROOT=str(root),
                                       DRONE_VISION_SNAPSHOT_DIR=str(root / 'vision')))
            console = FlightConsole(port=port); console.start()
            owner = 'coarse-pty-owner'
            state = lambda: console.status()['data']
            wait(lambda: state()['px4'].get('bidirectional'), 8)
            assert console.client.request('CLAIM', {}, owner)['ok']
            assert console.client.request('SET_MODE', {'mode': 'OFFBOARD'}, owner)['confirmed']
            assert console.client.request('ARM', {}, owner)['confirmed']
            fake.xyz[2] = -.5; fake.landed = 2
            wait(lambda: state()['px4'].get('landed_state') == 2)
            blocks = [dict(id='align', type='align', name='align', branches={}, timeout=15, next='distance',
                           params=dict(coarse_horizontal_threshold=80, coarse_settle_seconds=.1, lost_timeout=5)),
                      dict(id='distance', type='distance', name='distance', branches={}, timeout=15, next=None,
                           params=dict(duration=5, lost_timeout=2))]
            engine = MissionClient(console)
            engine.start(dict(version=1, id='coarse-test', name='coarse-test', entry='align', blocks=blocks), owner)
            wait(lambda: engine.status().get('current_id') == 'align', 5)
            time.sleep(.8)
            assert engine.status().get('current_id') == 'align', 'frozen frame caused handoff'
            mode[0] = 'blink'
            wait(lambda: engine.status().get('current_id') == 'distance', 8)
            events = engine.status()['events']
            assert any(e['event'] == 'vision_unavailable' for e in events), events
            assert any(e['event'] == 'coarse_alignment_handoff' for e in events), events
            assert sum(e.get('command') == 'VISION_START' for e in events) == 1
            engine.stop('hold'); wait(lambda: not engine.status()['active'], 5)
            print('PASS: frozen frame does not hand off; intermittent real frames hand off with one vision session')
        finally:
            done.set()
            if console: console.stop()
            if process:
                process.terminate()
                try: process.communicate(timeout=8)
                except subprocess.TimeoutExpired: process.kill(); process.communicate()
            if thread.is_alive(): thread.join(2)
            fake.close()
            if hasattr(fake, 'error'): raise fake.error


if __name__ == '__main__':
    run()
