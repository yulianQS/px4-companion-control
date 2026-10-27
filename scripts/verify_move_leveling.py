"""Relative-move leveling gate on a new PTY; no physical UART or flight dynamics."""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from verify_cpp_service import FakePX4, ServiceClient, wait


def run():
    binary = Path(sys.argv[1]).resolve()
    fake = FakePX4()
    assert fake.device.startswith('/dev/pts/')
    client = process = None
    with tempfile.TemporaryDirectory(prefix='move-leveling-pty-') as temp:
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
            fake.thread.start()
            # Mission validation reads only this sandbox's catalogue.
            root = Path(temp)
            (root/'config').mkdir()
            (root/'config/mission_blocks.json').write_bytes(
                (Path(__file__).resolve().parents[1]/'config/mission_blocks.json').read_bytes())
            process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port),
                                        '--logs', temp], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       env=dict(os.environ, DRONE_PROJECT_ROOT=temp))
            client = ServiceClient(port=port); client.start()
            owner = 'move-leveling-pty-owner'
            state = lambda: client.status()['data']
            command = lambda name, params=None: client.request(name, params or {}, owner)
            wait(lambda: state()['px4'].get('bidirectional'), 8)
            assert command('CLAIM')['ok']
            assert command('SET_MODE', {'mode':'OFFBOARD'})['confirmed']
            assert command('ARM')['confirmed']
            fake.xyz = [0., 0., -.5]; fake.landed = 2
            wait(lambda: state()['px4'].get('landed_state') == 2)

            def motion(): return state()['motion']
            def start(direction='back', **params):
                doc = dict(version=1, id='level-test', name='level-test', entry='move', blocks=[
                    dict(id='move', type='move', name='move', next=None, branches={}, timeout=8,
                         params=dict(direction=direction, distance=.3, speed=.6, acceleration=0,
                                     tolerance=.03, **params))])
                result = command('MISSION_START', doc)
                assert result['ok'], result
                wait(lambda: state()['mission'].get('active') and motion().get('state') == 'running'
                     and motion().get('phase') == 'leveling')
                return len(fake.setpoints)
            def held(first):
                points = [m for _, m in fake.setpoints[first:]]
                assert len(points) >= 2
                assert all(m.type_mask & 3 == 3 and not m.type_mask & 4 for m in points)
                assert all(abs(m.vx)+abs(m.vy)+abs(m.vz) < 1e-6 for m in points)
                assert all(abs(m.z + .5) < .001 for m in points)
            def finished(expected):
                wait(lambda: not state()['mission'].get('active'), 6)
                result = state()['mission']
                assert result['state'] == expected, result
                return result

            # Legacy direct MOVE retains its existing interface and does not gain a gate.
            fake.pitch = .8
            result = command('MOVE', dict(kind='move', direction='forward', distance=.1,
                                         speed=.2, acceleration=0, tolerance=.02))
            assert result['stage'] == 'running', result
            wait(lambda: motion().get('state') == 'succeeded')
            assert motion().get('phase') != 'leveling'

            # Mission moves always brake first. Roll and pitch must BOTH be small;
            # level attitude alone is insufficient while measured velocity is high.
            # Suspend the idealized follower while changing simulated yaw; the
            # prior completed task would otherwise immediately overwrite it.
            fake.follow_setpoints = False
            fake.heading = .4; fake.roll = .3
            wait(lambda: abs(state()['px4']['attitude_deg'][2] - .4*180/math.pi) < .1)
            first = start()
            fake.follow_setpoints = True
            time.sleep(.25); held(first)
            fake.pitch = 0
            time.sleep(.25); assert motion()['phase'] == 'leveling'; held(first)
            fake.roll = 0; fake.reported_velocity = [1., 0., 0.]
            time.sleep(.25); assert motion()['phase'] == 'leveling'; held(first)
            # Simulated braking drift must not count as requested backward travel.
            fake.xyz[0] += .4; fake.xyz[1] += .2
            expected_start = fake.xyz[:]
            fake.reported_velocity = [0., 0., 0.]
            time.sleep(.25); assert motion()['phase'] == 'leveling'
            fake.pitch = .2
            time.sleep(.2); assert motion()['phase'] == 'leveling'
            fake.pitch = 0
            time.sleep(.25); assert motion()['phase'] == 'leveling'
            try:
                wait(lambda: motion().get('phase') == 'moving', 3)
            except AssertionError:
                raise AssertionError(dict(motion=motion(), mission=state()['mission']))
            moved = motion()
            assert max(abs(a-b) for a,b in zip(moved['start_position'], expected_start)) < .001
            expected_goal = [expected_start[0]-.3*math.cos(.4), expected_start[1]-.3*math.sin(.4), -.5]
            assert max(abs(a-b) for a,b in zip(moved['goal_position'], expected_goal)) < .001
            result = finished('succeeded')
            assert motion()['state'] == 'succeeded'
            assert max(abs(a-b) for a,b in zip(fake.xyz, expected_goal)) < .03
            log = root/'mission_runs'/result['log_file']
            events = [json.loads(line) for line in log.read_text().splitlines()]
            assert {'leveling','moving'} <= {e.get('phase') for e in events if e['event']=='motion_feedback'}

            # Vertical relative moves use the same gate and keep their distance.
            initial_z = fake.xyz[2]
            start('up', level_settle_s=.1)
            finished('succeeded')
            assert abs(fake.xyz[2]-(initial_z-.3)) < .03

            # Frozen or nonfinite attitude must never unlock movement.
            fake.send_attitude = False
            start(level_settle_s=.5)
            finished('failed')
            assert motion()['phase'] == 'leveling'
            fake.send_attitude = True
            wait(lambda: state()['px4'].get('attitude_age_s', 9) < .1)
            fake.pitch = float('nan')
            start(level_timeout_s=1)
            finished('failed')
            assert motion()['phase'] == 'leveling'

            # Persistent tilt fails explicitly without ever starting the displacement.
            fake.pitch = .8
            start(level_timeout_s=1)
            result = finished('failed')
            assert '回正超时' in result['message'], result
            assert motion()['phase'] == 'leveling'
            # Cancellation remains responsive while waiting for level attitude.
            start()
            began = time.monotonic()
            assert command('MISSION_STOP', {'mode':'hold'})['ok']
            finished('stopped')
            assert time.monotonic()-began < 1
            print('PASS: mission leveling gate, tilt/velocity/continuous fresh feedback, drift rebasing, '
                  'height moves, unchanged raw MOVE, timeout, NaN/stale attitude, cancellation and phase logs')
        finally:
            if client: client.stop()
            if process:
                process.terminate(); out, err = process.communicate(timeout=8)
                assert process.returncode == 0, (out, err)
            fake.close()
            assert not hasattr(fake, 'error'), getattr(fake, 'error', None)


if __name__ == '__main__': run()
