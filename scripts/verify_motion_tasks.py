"""离线运动/任务/编码检查，不访问实机，不证明实际飞行稳定性。"""
import io
import math
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motion_tasks import MotionTasks, wrap, ZERO
from web_flight import FlightConsole, TelemetryLink, ControlError
from link import mavutil
from verify_web_control import FakeLink


def feedback(yaw=0, landed=2):
    return dict(connected=True, bidirectional=True, armed=True, mode='OFFBOARD',
        heartbeat_revision=1, heartbeat_age_s=.1, estimator_flags=46, estimator_age_s=.1,
        position_age_s=.05, attitude_age_s=.05, landed_age_s=.1, landed_state=landed,
        odometry_age_s=.05, odometry_reset_counter=0, position_sample_ms=1000,
        position_revision=1, local_position=dict(x=0., y=0., z=0., vx=0., vy=0., vz=0.),
        attitude_deg=[0., 0., float(yaw)], angular_rate_rad_s={'yawspeed': 0.},
        battery_age_s=.1, battery_v=16.)


def parameters(kind='move', direction='forward', **values):
    return dict(kind=kind, direction=direction, distance=90. if kind == 'rotate' else .5,
                speed=30. if kind == 'rotate' else .2,
                tolerance=3. if kind == 'rotate' else .05, timeout=30., **values)


class Tasks(unittest.TestCase):
    def drive(self, params, yaw=0, landed=2):
        engine, state = MotionTasks(), feedback(yaw, landed)
        engine.start(params, state, 0.)
        commands = []
        for step in range(1, 2401):
            now = step * .05
            command = engine.update(state, now)
            commands.append(command)
            position = state['local_position']
            for axis, speed in zip(('x', 'y', 'z'), command[:3]):
                position[axis] += speed * .05
                position['v' + axis] = speed
            state['attitude_deg'][2] = math.degrees(wrap(math.radians(state['attitude_deg'][2]) + command[3] * .05))
            state['angular_rate_rad_s']['yawspeed'] = command[3]
            state['position_sample_ms'] += 50
            state['position_revision'] += 1
            if params['kind'] == 'takeoff' and position['z'] < -.08:
                state['landed_state'] = 2
            if engine.task['state'] != 'running':
                break
        self.assertEqual(engine.task['state'], 'succeeded', engine.status(now))
        self.assertTrue(engine.task['target_reached'])
        return engine, state, commands

    def test_fixed_start_heading_all_directions(self):
        expected = {'forward': (0, 1), 'back': (0, -1), 'right': (-1, 0), 'left': (1, 0)}
        for direction, signs in expected.items():
            with self.subTest(direction=direction):
                _, state, commands = self.drive(parameters(direction=direction), yaw=90)
                x, y = state['local_position']['x'], state['local_position']['y']
                self.assertAlmostEqual(x, .5 * signs[0], delta=.06)
                self.assertAlmostEqual(y, .5 * signs[1], delta=.06)
                self.assertTrue(all(math.hypot(*cmd[:2]) <= .200001 for cmd in commands))

    def test_up_down_ned_sign_and_speed_limit(self):
        for direction, sign in (('up', -1), ('down', 1)):
            engine, state, commands = self.drive(parameters('height', direction))
            self.assertAlmostEqual(state['local_position']['z'], .5 * sign, delta=.06)
            self.assertTrue(all(abs(cmd[2]) <= .200001 for cmd in commands))

    def test_yaw_wrap_both_directions(self):
        for direction, sign, yaw in (('clockwise', 1, 179), ('counterclockwise', -1, -179)):
            params = parameters('rotate', direction)
            params.update(distance=30)
            engine, state, commands = self.drive(params, yaw=yaw)
            self.assertAlmostEqual(engine.task['relative_yaw_deg'], 30 * sign, delta=3)
            self.assertTrue(all(abs(cmd[3]) <= math.radians(30.001) for cmd in commands))

    def test_full_turn_is_accumulated_not_zero(self):
        params = parameters('rotate', 'clockwise')
        params.update(distance=360, timeout=30)
        engine, _, _ = self.drive(params)
        self.assertGreater(engine.task['measured'], 357)

    def test_takeoff_requires_actual_lift(self):
        engine, state, _ = self.drive(parameters('takeoff', 'up'), landed=1)
        self.assertEqual(state['landed_state'], 2)
        self.assertLess(state['local_position']['z'], -.44)

    def test_progress_and_timeout_are_not_time_simulation(self):
        engine, state = MotionTasks(), feedback()
        params = parameters()
        params.update(timeout=2)
        engine.start(params, state, 0)
        engine.update(state, 1)
        self.assertEqual(engine.task['progress'], 0)
        self.assertEqual(engine.update(state, 2.01), ZERO)
        self.assertEqual(engine.task['state'], 'timed_out')
        self.assertFalse(engine.task['target_reached'])

    def test_cancel_captures_real_current_position(self):
        engine, state = MotionTasks(), feedback()
        engine.start(parameters(), state, 0)
        state['local_position']['x'] = .13
        engine.cancel(state, .2)
        self.assertEqual(engine.task['state'], 'cancelled')
        self.assertAlmostEqual(engine.hold['position'][0], .13)
        self.assertEqual(engine.update(state, .25), ZERO)

    def test_old_velocity_bitmask_is_rejected(self):
        engine, state = MotionTasks(), feedback()
        state['estimator_flags'] = 12
        with self.assertRaises(ValueError):
            engine.start(parameters(), state, 0)

    def test_invalid_feedback_and_owner_are_latched(self):
        for change in ({'position_age_s': 2}, {'odometry_age_s': 2}, {'connected': False},
                       {'armed': False}, {'mode': 'LOITER'}, {'estimator_flags': 0},
                       {'odometry_reset_counter': 1}, {'position_sample_ms': 1}):
            with self.subTest(change=change):
                engine, state = MotionTasks(), feedback()
                engine.start(parameters(), state, 0)
                state.update(change)
                self.assertEqual(engine.update(state, .1), ZERO)
                self.assertEqual(engine.task['state'], 'failed')
                state.update(feedback())
                self.assertEqual(engine.update(state, .2), ZERO)
                self.assertEqual(engine.task['state'], 'failed')
        engine, state = MotionTasks(), feedback()
        engine.start(parameters(), state, 0)
        self.assertEqual(engine.update(state, .1, owner_active=False), ZERO)
        self.assertEqual(engine.task['state'], 'failed')

    def test_no_single_sample_completion(self):
        engine, state = MotionTasks(), feedback()
        engine.start(parameters(), state, 0)
        state['local_position']['x'] = .5
        engine.update(state, .1)
        engine.update(state, .7)
        self.assertEqual(engine.task['state'], 'running')

    def test_air_tasks_reject_ground_and_takeoff_rejects_air(self):
        with self.assertRaises(ValueError):
            MotionTasks().start(parameters(), feedback(landed=1), 0)
        with self.assertRaises(ValueError):
            MotionTasks().start(parameters('takeoff', 'up'), feedback(), 0)

    def test_parameters_reject_nonfinite_and_excess(self):
        for change in ({'distance': float('nan')}, {'speed': 5}, {'distance': True},
                       {'tolerance': 1}, {'direction': 'unknown'}, {'timeout': 121}):
            params = parameters()
            params.update(change)
            with self.assertRaises(ValueError):
                MotionTasks.parameters(params)

    def test_landing_not_mode_ack_completion(self):
        engine, state = MotionTasks(), feedback()
        engine.begin_land(0)
        engine.land_reply(True)
        state['mode'] = 'LAND'
        engine.update(state, .2)
        self.assertEqual(engine.task['state'], 'landing')
        state['landed_state'] = 1
        engine.update(state, .3)
        engine.update(state, 1.4)
        self.assertTrue(engine.task['needs_disarm'])
        self.assertEqual(engine.task['state'], 'landing')
        state['armed'] = False
        engine.update(state, 1.5)
        self.assertEqual(engine.task['state'], 'succeeded')

    def test_land_timeout_does_not_claim_landed(self):
        engine, state = MotionTasks(), feedback()
        engine.begin_land(0)
        engine.land_reply(True)
        state['mode'] = 'LAND'
        engine.update(state, 60.1)
        self.assertEqual(engine.task['state'], 'timed_out')
        self.assertFalse(engine.task['target_reached'])

    def test_zero_velocity_on_bad_yaw_jump(self):
        engine, state = MotionTasks(), feedback()
        engine.start(parameters('rotate', 'clockwise'), state, 0)
        state['attitude_deg'][2] = 90
        self.assertEqual(engine.update(state, .05), ZERO)
        self.assertEqual(engine.task['state'], 'failed')


class Integration(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.link.state.update(feedback(landed=1))
        self.console = FlightConsole(self.link)
        self.console.claim('motion-unit-client')
        self.console.zero_enabled = True
        self.console.zero_sent = time.monotonic()

    def action(self, **body):
        return self.console.perform('motion-unit-client', dict(request_id='motion-unit-request', **body))

    def test_takeoff_no_automatic_arm_and_no_bench_stop(self):
        self.console.bench_deadline = time.monotonic() + 10
        result = self.action(action='takeoff', task={'distance': .5, 'speed': .2})
        self.assertTrue(result['accepted'])
        self.assertFalse(result['confirmed'])
        self.assertEqual(self.link.commands, [])
        self.assertIsNone(self.console.bench_deadline)
        self.assertEqual(self.action(action='takeoff', task={'distance': .5, 'speed': .2}), result)

    def test_unarmed_start_rejected_without_setpoint(self):
        self.link.state['armed'] = False
        with self.assertRaises(ControlError):
            self.action(action='takeoff', task={'distance': .5, 'speed': .2})
        self.assertEqual(self.link.commands, [])
        self.assertIsNone(self.console.motion.task)

    def test_land_sends_normal_land_mode(self):
        self.link.state['landed_state'] = 2
        result = self.action(action='land')
        self.assertTrue(result['accepted'])
        self.assertEqual(self.link.commands, [(176, [1, 4, 6, 0, 0, 0, 0])])
        self.assertFalse(self.console.zero_enabled)
        self.assertEqual(self.console.motion.task['state'], 'landing')

    def test_airborne_disable_requests_land_not_disarm(self):
        self.link.state['landed_state'] = 2
        result = self.action(action='disable')
        self.assertFalse(result['control_disabled'])
        self.assertFalse(result['confirmed'])
        self.assertTrue(self.console.close_after_land)
        self.assertEqual(self.link.commands[0][0], 176)

    def test_wire_ned_velocity_and_yaw_rate(self):
        wire = io.BytesIO()
        link = TelemetryLink()
        link.target = (1, 1)
        link.connection = type('Connection', (), {'mav': mavutil.mavlink.MAVLink(wire)})()
        link.send_motion(.2, -.1, -.3, .4)
        msg = mavutil.mavlink.MAVLink(None).parse_char(wire.getvalue())
        self.assertEqual(msg.get_type(), 'SET_POSITION_TARGET_LOCAL_NED')
        self.assertEqual(msg.coordinate_frame, 1)
        self.assertEqual(msg.type_mask, 1479)
        self.assertAlmostEqual(msg.vx, .2, places=6)
        self.assertAlmostEqual(msg.vz, -.3, places=6)
        self.assertAlmostEqual(msg.yaw_rate, .4, places=6)
        self.assertFalse(msg.type_mask & 2048)


if __name__ == '__main__':
    unittest.main()
