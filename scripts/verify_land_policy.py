"""LAND/disable regression. Uses a new PTY and temporary CSVs, never UART7."""
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
    client = process = None
    with tempfile.TemporaryDirectory(prefix='land-policy-') as temp:
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            fake.thread.start()
            process = subprocess.Popen(
                [str(binary), '--device', fake.device, '--port', str(port), '--logs', temp],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            client = ServiceClient(port=port)
            client.start()
            owner = 'land-policy-test-owner'

            def state():
                return client.status()['data']

            def command(name, params=None):
                return client.request(name, params or {}, owner)

            def claim_offboard():
                assert command('CLAIM')['ok']
                assert command('SET_MODE', {'mode': 'OFFBOARD'})['confirmed']

            def assert_land_only(start):
                modes = [m for m in fake.commands[start:] if m.command == 176]
                assert modes and all((m.param2, m.param3) == (4, 6) for m in modes), modes
                assert fake.mode == (4, 6)

            wait(lambda: state()['px4'].get('bidirectional'), 8)
            assert not fake.commands and not fake.setpoints, 'Startup must remain read-only'

            # Ground disable must select LAND, even when already disarmed in OFFBOARD.
            claim_offboard()
            start = len(fake.commands)
            result = command('DISABLE')
            assert result.get('control_disabled') and result['confirmed'], result
            wait(lambda: not state()['control']['lease_active'])
            assert_land_only(start)
            assert not any(m.command == 400 for m in fake.commands[start:]), 'Do not disarm first'
            count = len(fake.setpoints)
            time.sleep(.4)
            assert len(fake.setpoints) == count, 'No Offboard setpoints after confirmed LAND'

            # Compatibility RELEASE and explicit LAND must obey the same mode policy.
            for name in ('RELEASE', 'LAND'):
                claim_offboard()
                start = len(fake.commands)
                result = command(name)
                assert result['confirmed'], result
                assert_land_only(start)

            # A rejected LAND must not claim control was disabled or choose another mode.
            claim_offboard()
            fake.mode_result = 2
            start = len(fake.commands)
            result = command('DISABLE')
            assert not result['confirmed'] and not result.get('control_disabled'), result
            assert result['stage'] == 'failed' and fake.mode == (6, 0)
            assert state()['control']['lease_active']
            assert all((m.param2, m.param3) == (4, 6)
                       for m in fake.commands[start:] if m.command == 176)
            fake.mode_result = 0

            # Airborne disable cancels joystick, waits for landing and then normal disarm.
            fake.land_automatically = False
            fake.armed, fake.landed, fake.xyz[2] = True, 2, -1.
            wait(lambda: state()['px4'].get('armed') is True)
            assert command('STICK_START')['confirmed']
            wait(lambda: state()['joystick']['active'])
            session = state()['joystick']['session']
            result = command('STICK_INPUT', {'session': session, 'sequence': 0,
                                            'axes': dict(throttle=0, yaw=0, pitch=0, roll=0)})
            assert result['ok'], result
            start = len(fake.commands)
            result = command('DISABLE')
            assert result['stage'] == 'running' and not result['control_disabled'], result
            wait(lambda: state()['motion']['state'] == 'landing'
                 and state()['motion'].get('land_confirmed')
                 and not state()['control']['zero_stream'])
            assert not state()['joystick']['active']
            assert state()['control']['lease_active']
            assert command('HOLD')['stage'] == 'failed'
            assert command('STICK_START')['stage'] == 'failed'
            assert command('STICK_STOP', {'session': session})['confirmed']
            assert not state()['control']['zero_stream']
            count = len(fake.setpoints)
            time.sleep(.4)
            assert len(fake.setpoints) == count
            assert_land_only(start)
            # Touchdown alone must not complete disable until an unarmed heartbeat arrives.
            fake.xyz[2], fake.landed = 0., 1
            wait(lambda: state()['px4'].get('landed_state') == 1)
            assert state()['motion']['state'] == 'landing'
            wait(lambda: state()['motion']['state'] == 'succeeded', 5)
            wait(lambda: not state()['control']['lease_active'])
            assert state()['px4']['armed'] is False
            assert_land_only(start)
            commands = fake.commands[start:]
            assert commands[0].command == 176
            assert any(m.command == 400 and m.param1 == 0 for m in commands)

            # LAND also supersedes an active position task, not just joystick control.
            claim_offboard()
            fake.armed, fake.landed, fake.xyz[2] = True, 2, -1.
            wait(lambda: state()['px4'].get('armed') is True)
            result = command('SET_POSITION', {'frame': 'LOCAL_NED', 'unit': 'cm',
                                               'x': 150, 'y': 0, 'z': -100})
            assert result['accepted'], result
            old_task = result['id']
            start = len(fake.commands)
            result = command('LAND')
            assert result['stage'] == 'running', result
            wait(lambda: state()['motion'].get('id') == result['id']
                 and state()['motion'].get('land_confirmed'))
            assert any(e['id'] == old_task and e['message'] == 'Superseded by LAND'
                       for e in state()['command_events'])
            # Even a disarmed heartbeat cannot permit another mode before touchdown.
            fake.armed = False
            wait(lambda: state()['px4'].get('armed') is False)
            assert command('SET_MODE', {'mode': 'LOITER'})['stage'] == 'failed'
            assert_land_only(start)
            fake.landed, fake.xyz[2] = 1, 0.
            wait(lambda: state()['motion']['state'] == 'succeeded')

            # No heartbeat: no false completion and no mode/arming command is sent.
            assert command('CLAIM')['ok']
            fake.send_heartbeat = False
            wait(lambda: not state()['px4']['connected'], 5)
            start = len(fake.commands)
            assert command('DISABLE')['stage'] == 'failed'
            assert command('LAND')['stage'] == 'failed'
            assert len(fake.commands) == start
            print('PASS: ground/airborne DISABLE, RELEASE, LAND, rejection, heartbeat loss, '
                  'joystick cancellation, no post-LAND setpoints or mode fallback')
        finally:
            if client:
                client.stop()
            if process:
                process.terminate()
                out, err = process.communicate(timeout=8)
                assert process.returncode == 0, (out, err)
            fake.close()
            assert not hasattr(fake, 'error'), getattr(fake, 'error', None)


if __name__ == '__main__':
    run()
