"""Non-flight integration test: a PTY MAVLink peer, never the physical UART.

Run on Linux with the existing pymavlink venv. Every ARM/mode/setpoint in this
test goes ONLY to a newly allocated /dev/pts/N pseudoterminal.
"""
import csv
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
import uuid
os.environ['MAVLINK20'] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink import mavutil
from web.server.cpp_bridge import ServiceClient
from web.server.flight_analysis import analyze_csv


def wait(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('condition timed out')


class FakePX4:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.device = os.ttyname(self.slave)
        assert self.device.startswith('/dev/pts/'), 'Test must never open physical UART'
        os.set_blocking(self.master, False)
        self.done = threading.Event()
        self.armed = False
        self.ground_z = 0.
        self.follow_setpoints = True
        self.mode = (4, 3)
        self.landed = 1
        self.xyz = [0., 0., 0.]
        self.heading = 0.
        self.roll = self.pitch = 0.
        self.reported_velocity = [0., 0., 0.]
        self.send_attitude = True
        self.epoch = 0
        self.flags = 302
        self.setpoints = []
        self.commands = []
        self.delay_ack = 0
        self.mode_result = 0
        self.range_cm = None
        self.landed_interval = .05
        self.send_landed = True
        self.land_automatically = True
        self.send_heartbeat = True
        self.param_sets = []
        self.height_write_mode = "accept"
        self.takeoff_height = .1
        self.takeoff_speed = .2
        self.takeoff_automatically = True
        self.takeoff_z = 0.
        self.takeoff_result = 0
        self.takeoff_changes_mode = True
        self.takeoff_drop_ack = False
        self.send_native_target = False
        self.native_target_mask = 0
        self.native_target_override = None
        self.takeoff_absolute_z = None
        self.takeoff_landed_state = 2
        self.offboard_result = 0
        self.boot = time.monotonic()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def write(self, data):
        return os.write(self.master, data)

    def run(self):
        mav = mavutil.mavlink.MAVLink(self, srcSystem=1, srcComponent=1)
        parser = mavutil.mavlink.MAVLink(None)
        sent, hb, landed_sent = 0, 0, 0
        pending = []
        try:
            while not self.done.is_set():
                now = time.monotonic()
                if select.select([self.master], [], [], .01)[0]:
                    for byte in os.read(self.master, 8192):
                        msg = parser.parse_char(bytes([byte]))
                        if not msg:
                            continue
                        if msg.get_type() == 'PARAM_REQUEST_READ':
                            name = msg.param_id
                            if isinstance(name, str): name = name.encode()
                            values = {b'MAV_SYS_ID': (1., 6), b'MIS_TAKEOFF_ALT': (self.takeoff_height, 9),
                                      b'MPC_TKO_SPEED': (self.takeoff_speed, 9)}
                            if name in values:
                                value, kind = values[name]
                                mav.param_value_send(name, value, kind, 3, 0)
                        elif msg.get_type() == 'PARAM_SET':
                            self.param_sets.append(msg)
                            name = msg.param_id.encode() if isinstance(msg.param_id, str) else msg.param_id
                            if name == b'MIS_TAKEOFF_ALT':
                                if self.height_write_mode == 'accept':
                                    self.takeoff_height = msg.param_value
                                if self.height_write_mode != 'drop':
                                    mav.param_value_send(name, self.takeoff_height, 9, 3, 0)
                        elif msg.get_type() == 'COMMAND_LONG':
                            self.commands.append(msg)
                            ack = self.mode_result if msg.command == 176 else self.takeoff_result if msg.command == 22 else 0
                            if msg.command == 176 and (int(msg.param2), int(msg.param3)) == (6, 0):
                                ack = self.offboard_result or ack
                            if msg.command == 176 and ack == 0:
                                self.mode = (int(msg.param2), int(msg.param3))
                            if msg.command == 22 and ack == 0 and self.takeoff_changes_mode:
                                self.mode = (4, 2)
                                self.takeoff_z = self.xyz[2] - self.takeoff_height
                                if self.takeoff_absolute_z is not None:
                                    self.takeoff_z = self.takeoff_absolute_z
                            if msg.command == 400:
                                self.armed = bool(msg.param1)
                                if self.armed:self.ground_z = self.xyz[2]
                            if not (msg.command == 22 and self.takeoff_drop_ack):
                                pending.append((now + self.delay_ack, msg.command, ack))
                        elif msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
                            self.setpoints.append((now, msg))
                            if self.armed and self.mode == (6, 0) and self.follow_setpoints:
                                for i, key in enumerate(('x', 'y', 'z')):
                                    if not msg.type_mask & (1 << i):
                                        self.xyz[i] = getattr(msg, key)
                                    else:
                                        self.xyz[i] += getattr(msg, ('vx', 'vy', 'vz')[i]) * .05
                                if not msg.type_mask & 1024:
                                    self.heading = msg.yaw
                                elif not msg.type_mask & 2048:
                                    self.heading += msg.yaw_rate * .05
                                if self.ground_z - self.xyz[2] > .025:
                                    self.landed = 2
                for item in pending[:]:
                    if now >= item[0]:
                        mav.command_ack_send(item[1], item[2], 0, 0, 42, 191)
                        pending.remove(item)
                if self.mode == (4, 2) and self.takeoff_automatically and self.armed:
                    self.xyz[2] = max(self.takeoff_z, self.xyz[2] - self.takeoff_speed * .01)
                    self.landed = self.takeoff_landed_state
                if self.mode == (4, 6) and self.land_automatically:
                    self.xyz[2] = 0
                    self.landed = 1
                    self.armed = False
                if now - sent >= .05:
                    ms = max(0, int((now - self.boot) * 1000))
                    if self.send_attitude:
                        mav.attitude_send(ms, self.roll, self.pitch, self.heading, 0, 0, 0)
                    mav.odometry_send(ms * 1000, 1, 12, *self.xyz, [1, 0, 0, 0], 0, 0, 0, 0, 0, 0, [0.] * 21, [0.] * 21, self.epoch, 2, 100)
                    mav.local_position_ned_send(ms, *self.xyz, *self.reported_velocity)
                    if self.send_native_target:
                        z = self.takeoff_z if self.native_target_override is None else self.native_target_override
                        mav.position_target_local_ned_send(ms, 1, self.native_target_mask,
                                                          self.xyz[0], self.xyz[1], z,
                                                          0, 0, 0, 0, 0, 0, self.heading, 0)
                    mav.estimator_status_send(ms * 1000, self.flags, 0, 0, 0, 0, 0, 0, .01, .01)
                    if self.send_landed and now - landed_sent >= self.landed_interval:
                        mav.extended_sys_state_send(0, self.landed)
                        landed_sent = now
                    if self.range_cm is not None:
                        mav.distance_sensor_send(ms, 10, 400, self.range_cm, 0, 0, 25, 255)
                    mav.sys_status_send(0, 0, 0, 100, 16800, 100, 90, 0, 0, 0, 0, 0, 0)
                    sent = now
                if self.send_heartbeat and now - hb >= .2:
                    main, sub = self.mode
                    mav.heartbeat_send(2, 12, 1 | (128 if self.armed else 0), main << 16 | sub << 24, 3)
                    hb = now
        except Exception as error:
            self.error = error
            self.done.set()

    def close(self):
        self.done.set()
        self.thread.join(2)
        os.close(self.master)
        os.close(self.slave)


def run():
    binary = Path(sys.argv[1] if len(sys.argv) > 1 else 'build/drone_service').resolve()
    fake = FakePX4()
    client = None
    process = None
    with tempfile.TemporaryDirectory(prefix='cpp-flight-test-') as temp:
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            fake.thread.start()
            process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port), '--logs', temp], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            client = ServiceClient(port=port)
            client.start()
            wait(lambda: client.status()['data']['px4'].get('bidirectional'), 8)
            owner = 'offline-test-client'
            expected_task = None

            def command(name, params=None, request_id=None):
                nonlocal expected_task
                result = client.request(name, params or {}, owner, request_id)
                if result.get('stage') == 'running':
                    expected_task = result['id']
                return result

            def state():
                return client.status()['data']

            def completed():
                motion = state().get('motion', {})
                return motion.get('id') == expected_task and motion.get('state') == 'succeeded'

            assert not fake.commands and not fake.setpoints, 'Startup must remain monitor-only'
            assert command('ARM')['stage'] == 'failed', 'No control lease'
            assert command('CLAIM')['ok']
            assert command('ARM')['stage'] == 'failed', 'No OFFBOARD'
            fake.delay_ack = 1
            before = len(fake.setpoints)
            result = command('SET_MODE', {'mode': 'OFFBOARD'})
            assert result['confirmed'], result
            assert len(fake.setpoints) - before >= 35, 'ACK wait must not block 20Hz loop'
            fake.delay_ack = 0
            rid = uuid.uuid4().hex
            result = command('ARM', request_id=rid)
            assert result['confirmed'], result
            armed_count = sum(msg.command == 400 and msg.param1 == 1 for msg in fake.commands)
            assert command('ARM', request_id=rid)['confirmed']
            assert sum(msg.command == 400 and msg.param1 == 1 for msg in fake.commands) == armed_count
            assert command('DISARM', request_id=rid)['stage'] == 'failed', 'ID cannot be reused'
            assert command('TAKEOFF', {'distance': .1, 'speed': .2, 'tolerance': .02, 'timeout': 10})['stage'] == 'running'
            wait(completed, 5)
            assert state()['control']['setpoint']['position'][2] < -.09
            assert command('SET_POSITION', {'frame': 'LOCAL_NED', 'unit': 'cm', 'x': 10, 'y': 0, 'z': -10, 'timeout': 10})['accepted']
            wait(completed, 5)
            assert abs(fake.xyz[0] - .1) < .02, 'cm conversion must be C++'
            first=len(fake.setpoints)
            result=command('MOVE',dict(kind='move',direction='forward',distance=1,speed=2,acceleration=0,timeout=10))
            assert result['stage']=='running'
            wait(lambda:state()['motion'].get('id')==result['id'] and completed(),5)
            assert max(abs(m.vx) for _,m in fake.setpoints[first:])>=1.99,'Requested 2 m/s must not be clipped'
            first=len(fake.setpoints)
            result=command('SET_VELOCITY',dict(frame='LOCAL_NED',vx=2,duration=.3))
            assert result['stage']=='running'
            wait(lambda:state()['motion'].get('id')==result['id'] and completed(),3)
            assert max(abs(m.vx) for _,m in fake.setpoints[first:])>=1.99
            assert command('MOVE', {'kind': 'rotate', 'direction': 'clockwise', 'distance': 10, 'speed': 30, 'tolerance': 1, 'timeout': 10})['stage'] == 'running'
            wait(completed, 5)
            assert command('SET_VELOCITY', {'frame': 'ambiguous', 'vx': .1})['stage'] == 'failed'
            assert command('SET_VELOCITY', {'frame': 'BODY_FRD', 'vx': .1, 'duration': .4})['stage'] == 'running'
            wait(completed, 3)
            assert command('HOLD')['confirmed']
            fixed = state()['control']['setpoint']['position']
            time.sleep(.3)
            assert state()['control']['setpoint']['position'] == fixed, 'HOLD point must not follow measured drift'
            assert command('STICK_START')['confirmed']
            session = state()['joystick'].get('session')
            wait(lambda: bool(state()['joystick'].get('active')))
            session = state()['joystick']['session']
            neutral = dict(throttle=0, yaw=0, pitch=0, roll=0)
            assert command('STICK_INPUT', {'session': session, 'sequence': 0, 'axes': neutral})['ok']
            assert command('STICK_INPUT', {'session': session, 'sequence': 0, 'axes': neutral})['stage'] == 'failed'
            assert command('STICK_INPUT', {'session': session, 'sequence': 1, 'axes': dict(neutral, pitch=1)})['ok']
            wait(lambda: not state()['joystick'].get('active'), 3)
            assert state()['control']['source'] == 'hold'
            assert command('EXECUTE_TRAJECTORY', {'trajectory': 'circle', 'radius': 1, 'angular_speed_deg': 30, 'height': .1})['stage'] == 'running'
            assert command('HOLD')['confirmed']
            assert command('EXECUTE_TRAJECTORY', {'trajectory': 'circle', 'radius': .1, 'angular_speed_deg': 60, 'turns': 1, 'height': .1, 'timeout': 20})['stage'] == 'running'
            command('RENEW')
            wait(completed, 12)
            for trajectory, params in [('line', {'length': .1}), ('rectangle', {'length': .1, 'width': .1}), ('waypoints', {'frame': 'LOCAL_NED', 'unit': 'cm', 'waypoints': [[0, 0, -10]]}), ('hover', {'duration': 1})]:
                command('RENEW')
                result = command('EXECUTE_TRAJECTORY', dict(params, trajectory=trajectory, height=.1, timeout=20))
                assert result['stage'] == 'running', result
                wait(completed, 12)
            assert command('SET_POSITION', {'frame': 'LOCAL_NED', 'unit': 'cm', 'x': 50, 'y': 0, 'z': -10})['accepted']
            assert command('STOP_TRAJECTORY')['confirmed']
            assert state()['control']['zero_stream'], 'STOP must keep Offboard setpoint stream'
            command('SET_POSITION', {'frame': 'LOCAL_NED', 'unit': 'cm', 'x': 50, 'y': 0, 'z': -10})
            fake.epoch += 1
            wait(lambda: state()['px4'].get('odometry_reset_counter') == fake.epoch)
            assert state()['motion'].get('state') in ('running', 'succeeded'), 'Counter alone must not abort NED motion'
            fake.boot = time.monotonic()
            wait(lambda: state()['motion'].get('state') == 'failed')
            assert state()['control']['setpoint']['position'] == [None, None, None], 'Reset must not retain old coordinate goal'
            command('RENEW')
            result = command('LAND')
            assert result.get('stage') in ('running', 'completed'), result
            if result.get('stage') == 'completed':
                expected_task = result['id']
            wait(completed)
            wait(lambda: bool(list(Path(temp).glob('flight_*.csv'))))
            files = [p for p in Path(temp).glob('flight_*.csv') if '.partial.' not in p.name]
            assert len(files) == 1, list(Path(temp).iterdir())
            rows = list(csv.DictReader(files[0].open()))
            assert len(rows) > 50
            assert {'timestamp','target_x','actual_x','vx'} <= set(rows[0])
            replay = analyze_csv(files[0].read_text(), files[0].name)
            assert replay['analysis']['compared_points'] > 10, replay['analysis']
            assert replay['analysis']['segments'] >= 2, 'EKF reset must break segments'
            assert command('DISABLE')['control_disabled']
            print('PASS: PTY-only C++ MAVLink/TCP/ACK isolation/dedup/position-cm/velocity/joystick/5 trajectories/HOLD/reset/LAND/CSV/RMSE')
        finally:
            if client:
                client.stop()
            if process:
                process.terminate()
                try:
                    out, err = process.communicate(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    out, err = process.communicate()
                if process.returncode not in (0, -15):
                    print('SERVICE:', out, err)
            fake.close()
            if hasattr(fake, 'error'):
                print('FAKE PX4:', fake.error)


if __name__ == "__main__":
    run()
