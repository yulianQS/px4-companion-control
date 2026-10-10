"""离线双摇杆与控制来源协调验证；不访问UART、不驱动电机。"""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joystick_control import JoystickControl
from motion_tasks import ZERO
from web_flight import FlightConsole, ControlError
from web_control import Handler, ThreadingHTTPServer
from verify_web_control import FakeLink
from verify_motion_tasks import feedback, parameters


def axes(**values):
    return dict(dict(throttle=0., yaw=0., pitch=0., roll=0.), **values)


class Sticks(unittest.TestCase):
    def ready(self, yaw=0):
        engine, state = JoystickControl(), feedback(yaw=yaw)
        engine.start(state, 0)
        engine.receive(dict(session=engine.session, sequence=0, axes=axes()), 0)
        return engine, state

    def send(self, engine, now, **values):
        engine.receive(dict(session=engine.session, sequence=engine.sequence + 1, axes=axes(**values)), now)

    def drive(self, **values):
        engine, state = self.ready()
        for i in range(1, 21):
            self.send(engine, i * .05, **values)
            target = engine.update(state, i * .05)
        return target

    def test_axis_directions_and_limits(self):
        for name, index, sign in [('pitch', 0, 1), ('roll', 1, 1), ('throttle', 2, -1), ('yaw', 3, 1)]:
            for direction in (-1, 1):
                command = self.drive(**{name: direction})
                self.assertGreater(command[index] * sign * direction, 0)
                self.assertLessEqual(abs(command[index]), .2 if index < 3 else math.radians(30))
                for other in set(range(4)) - {index}:
                    self.assertAlmostEqual(command[other], 0)

    def test_current_heading_conversion(self):
        engine, state = self.ready(yaw=90)
        self.send(engine, .05, pitch=1)
        command = engine.update(state, .05)
        self.assertAlmostEqual(command[0], 0)
        self.assertGreater(command[1], 0)
        state['attitude_deg'][2] = 180
        self.send(engine, .1, pitch=1)
        # 充分时间后机头向后，前推应为局部-X；坐标在开发板更新。
        for i in range(3, 31):
            self.send(engine, i * .05, pitch=1)
            command = engine.update(state, i * .05)
        self.assertLess(command[0], 0)
        self.assertAlmostEqual(command[1], 0)

    def test_diagonal_vector_cap_and_deadzone(self):
        command = self.drive(pitch=1, roll=1, throttle=1, yaw=1)
        self.assertLessEqual(math.hypot(*command[:2]), .2 + 1e-10)
        self.assertEqual(self.drive(pitch=.03, roll=.03, throttle=.03, yaw=.03), ZERO)

    def test_left_and_right_release_independently(self):
        engine, state = self.ready()
        self.send(engine, .05, throttle=1, pitch=1)
        engine.update(state, .05)
        self.send(engine, .1, pitch=1)
        command = engine.update(state, .1)
        self.assertGreater(command[0], 0)
        self.assertEqual(command[2:], (0., 0.))
        self.send(engine, .15)
        self.assertEqual(engine.update(state, .15), ZERO)

    def test_timeout_latches_and_late_receive_cannot_revive(self):
        engine, state = self.ready()
        self.send(engine, .1, pitch=1)
        engine.update(state, .1)
        self.assertEqual(engine.update(state, .61), ZERO)
        self.assertFalse(engine.active)
        with self.assertRaises(ValueError):
            self.send(engine, .62, pitch=1)
        engine, state = self.ready()
        with self.assertRaises(ValueError):
            self.send(engine, .51, pitch=1)
        self.assertFalse(engine.active)

    def test_held_throttle_independent_of_yaw_release_and_cleared_on_stop(self):
        engine, state = self.ready()
        for i in range(1, 21):
            self.send(engine, i * .05, throttle=.5, yaw=.5)
            command = engine.update(state, i * .05)
        vertical = command[2]
        self.send(engine, 1.05, throttle=.5)
        command = engine.update(state, 1.05)
        self.assertAlmostEqual(command[2], vertical)
        self.assertEqual(command[3], 0)
        self.assertLess(command[2], 0)
        self.assertEqual(engine.update(state, 1.56), ZERO)
        self.assertEqual(engine.axes, axes())

    def test_first_frame_neutral_sequence_and_session(self):
        engine, state = JoystickControl(), feedback()
        engine.start(state, 0)
        with self.assertRaises(ValueError):
            self.send(engine, .05, pitch=1)
        self.send(engine, .1)
        for body in [dict(session=engine.session, sequence=engine.sequence, axes=axes()),
                     dict(session='previous-session', sequence=3, axes=axes()),
                     dict(session=engine.session, sequence=True, axes=axes())]:
            with self.assertRaises(ValueError):
                engine.receive(body, .15)
        self.assertEqual(engine.received_at, .1)

    def test_bad_input_never_refreshes_timeout(self):
        engine, state = self.ready()
        for value in (True, float('nan'), float('inf'), 1.01, -1.01, '1'):
            with self.assertRaises(ValueError):
                self.send(engine, .2, pitch=value)
        self.assertEqual(engine.received_at, 0)

    def test_heading_and_connection_required_but_ground_and_position_reset_allowed(self):
        for change in ({'connected': False}, {'mode': 'LAND'}, {'attitude_age_s': 3},
                       {'attitude_deg': [0, 0, float('nan')]}):
            engine, state = JoystickControl(), feedback()
            state.update(change)
            with self.assertRaises(ValueError):
                engine.start(state, 0)
        for change in ({'armed': False, 'landed_state': 1}, {'position_age_s': 3},
                       {'odometry_reset_counter': 1, 'odometry_age_s': None, 'estimator_flags': 0}):
            engine, state = self.ready()
            self.send(engine, .1, pitch=1)
            state.update(change)
            self.assertGreater(engine.update(state, .1)[0], 0)
            self.assertTrue(engine.active)
        for change in ({'connected': False}, {'mode': 'LOITER'}):
            engine, state = self.ready()
            self.send(engine, .1, pitch=1)
            state.update(change)
            self.assertEqual(engine.update(state, .1), ZERO)
            self.assertFalse(engine.active)
        engine, state = self.ready()
        self.assertEqual(engine.update(state, .1, owner_active=False), ZERO)
        self.assertFalse(engine.active)


class RecordingLink(FakeLink):
    def __init__(self):
        super().__init__()
        self.state.update(feedback())
        self.targets = []

    def send_motion(self, *values):
        self.targets.append(values)


class Coordination(unittest.TestCase):
    def setUp(self):
        self.link = RecordingLink()
        self.console = FlightConsole(self.link)
        self.console.claim('joystick-unit-client')
        self.console.zero_enabled = True
        self.console.zero_sent = time.monotonic()
        self.sequence = 0

    def action(self, **body):
        self.sequence += 1
        return self.console.perform('joystick-unit-client', dict(request_id='joystick-request-%s' % self.sequence, **body))

    def start(self):
        result = self.action(action='stick_start')
        self.console.stick_input('joystick-unit-client', dict(session=result['session'], sequence=0, axes=axes()))
        return result

    def test_unarmed_ground_start_streams_neutral_without_arming(self):
        self.link.state.update(armed=False, landed_state=1)
        self.start()
        self.assertEqual(self.link.commands, [])
        self.assertTrue(self.console.joystick.active)
        self.assertEqual(self.link.targets, [])
        self.console._stream_once()
        self.assertEqual(self.link.targets, [ZERO])
        self.assertFalse(self.link.state['armed'])

    def test_takeover_cancels_old_goal_and_blocks_new_task(self):
        self.console.motion.start(parameters(), self.link.status(), time.monotonic())
        self.start()
        self.assertEqual(self.console.motion.task['state'], 'cancelled')
        self.assertIsNone(self.console.motion.hold)
        with self.assertRaises(ControlError):
            self.action(action='task_start', task=parameters())
        self.assertEqual(self.console.status()['data']['control']['source'], 'joystick')
        self.assertEqual(self.link.commands, [])

    def test_takeoff_can_be_taken_over_but_land_keeps_its_source(self):
        self.console.motion.start(parameters('takeoff', 'up'), feedback(landed=1), time.monotonic())
        result = self.start()
        self.assertEqual(self.console.motion.task['state'], 'cancelled')
        self.action(action='stick_stop', session=result['session'])
        self.console.motion.begin_land(time.monotonic())
        with self.assertRaises(ControlError):
            self.start()

    def test_manual_arm_clears_ground_debug_input_without_bench_timer(self):
        self.link.state.update(armed=False, landed_state=1)
        result = self.start()
        self.console.stick_input('joystick-unit-client', dict(session=result['session'], sequence=1, axes=axes(pitch=1)))
        reply = self.action(action='arm')
        self.assertTrue(reply['confirmed'])
        self.assertFalse(self.console.joystick.active)
        self.assertEqual(self.console.joystick.last_command, ZERO)
        self.assertIsNone(self.console.bench_deadline)
        self.assertEqual(self.link.commands, [(400, [1, 0, 0, 0, 0, 0, 0])])

    def test_stop_holds_new_position_and_late_stop_cannot_break_new_task(self):
        result = self.start()
        self.link.state['local_position']['x'] = 2.
        self.action(action='stick_stop', session=result['session'])
        self.assertFalse(self.console.joystick.active)
        self.assertEqual(self.console.motion.hold['position'][0], 2.)
        self.console.motion.start(parameters(), self.link.status(), time.monotonic())
        goal = copy.deepcopy(self.console.motion.hold)
        self.action(action='stick_stop', session=result['session'])
        self.assertEqual(self.console.motion.hold, goal)
        self.assertEqual(self.console.motion.task['state'], 'running')

    def test_land_preempts_and_rejects_old_frames(self):
        result = self.start()
        self.action(action='land')
        self.assertFalse(self.console.joystick.active)
        self.assertEqual(self.link.commands, [(176, [1, 4, 6, 0, 0, 0, 0])])
        with self.assertRaises(ControlError):
            self.console.stick_input('joystick-unit-client', dict(session=result['session'], sequence=1, axes=axes(pitch=1)))

    def test_only_stream_thread_writes_and_expiry_does_not_resume_goal(self):
        self.console.motion.start(parameters(), self.link.status(), time.monotonic())
        result = self.start()
        self.console.stick_input('joystick-unit-client', dict(session=result['session'], sequence=1, axes=axes(pitch=1)))
        self.assertEqual(self.link.targets, [])
        self.console._stream_once()
        self.assertGreater(self.link.targets[-1][0], 0)
        self.console.joystick.received_at = time.monotonic() - 1
        self.console._stream_once()
        self.assertEqual(self.link.targets[-1], ZERO)
        self.assertFalse(self.console.joystick.active)
        self.assertEqual(self.console.motion.task['state'], 'cancelled')

    def test_fast_input_endpoint_owner_origin_and_order(self):
        result = self.start()
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.console = self.console
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            base = 'http://127.0.0.1:' + str(server.server_port)
            body = dict(session=result['session'], sequence=1, axes=axes(pitch=1))
            def request(extra=None):
                headers = {'Content-Type': 'application/json', 'X-Control-Request': '1', 'X-Client-ID': 'joystick-unit-client'}
                headers.update(extra or {})
                return Request(base + '/api/control/stick', data=json.dumps(body).encode(), headers=headers)
            for headers in ({'X-Client-ID': 'another-client'}, {'Origin': 'http://evil.test'}):
                with self.assertRaises(HTTPError) as caught:
                    urlopen(request(headers), timeout=2)
                self.assertEqual(caught.exception.code, 403)
            with urlopen(request(), timeout=2) as response:
                self.assertEqual(json.load(response)['sequence'], 1)
            with self.assertRaises(HTTPError) as caught:
                urlopen(request(), timeout=2)
            self.assertEqual(caught.exception.code, 409)
            self.assertEqual(self.link.targets, [])
        finally:
            server.shutdown(); server.server_close(); worker.join(2)


if __name__ == '__main__':
    unittest.main()
