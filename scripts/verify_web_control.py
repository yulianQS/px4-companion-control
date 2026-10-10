"""离线验证真实控制边界；只用 FakeLink，不访问串口或电机。"""
import copy
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_flight import ControlError, FlightConsole, MODES
from web_control import Handler, ThreadingHTTPServer


class FakeLink:
    def __init__(self):
        self.state = dict(connected=True, bidirectional=True, armed=False, mode='OFFBOARD',
            heartbeat_revision=1, heartbeat_age_s=0, estimator_flags=46, estimator_age_s=.1,
            position_age_s=.1, battery_v=16.5, battery_age_s=.1, landed_state=1, landed_age_s=.1)
        self.commands = []
        self.ack = 0
        self.timeout = False
        self.confirm = True

    def status(self):
        return copy.deepcopy(self.state)

    def command(self, number, params, timeout=3):
        self.commands.append((number, params))
        if self.ack == 0 and self.confirm:
            self.state['heartbeat_revision'] += 1
            if number == 400:
                self.state['armed'] = bool(params[0])
            elif number == 176:
                self.state['mode'] = next(name for name, pair in MODES.items() if pair == tuple(params[1:3]))
        if self.timeout:
            raise TimeoutError('注入 ACK 丢失')
        return self.ack

    def send_velocity(self, *values):
        assert values == (0, 0, 0)

    def stop(self):
        pass


class Boundaries(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.console = FlightConsole(self.link)
        self.console.claim('client-unit-test')
        self.console.zero_enabled = True
        self.console.zero_sent = time.monotonic()

    def action(self, **body):
        return self.console.perform('client-unit-test', dict(request_id='request-unit-test', **body))

    def test_switch_off_never_sends_arm(self):
        self.console.lease_until = 0
        with self.assertRaises(ControlError):
            self.action(action='arm')
        self.assertEqual(self.link.commands, [])

    def test_arm_preconditions(self):
        for change in ({'connected': False}, {'armed': True}, {'mode': 'LOITER'},
                       {'estimator_flags': 0}, {'estimator_age_s': 20},
                       {'landed_state': 2}, {'landed_age_s': 20},
                       {'battery_v': 13}, {'battery_age_s': 20}):
            with self.subTest(change=change):
                previous = self.link.status()
                self.link.state.update(change)
                with self.assertRaises(ControlError):
                    self.action(action='arm')
                self.assertEqual(self.link.commands, [])
                self.link.state = previous

    def test_ordinary_arm_and_idempotency(self):
        body = dict(action='arm')
        first = self.action(**body)
        self.assertTrue(first['confirmed'])
        self.assertEqual(self.action(**body), first)
        self.assertEqual(self.link.commands, [(400, [1, 0, 0, 0, 0, 0, 0])])
        self.assertIsNone(self.console.bench_deadline)
        with self.assertRaises(ControlError):
            self.action(action='disarm')

    def test_ack_loss_with_new_heartbeat(self):
        self.link.timeout = True
        result = self.action(action='mode', mode='LOITER')
        self.assertTrue(result['confirmed'])
        self.assertIsNone(result['ack'])

    def test_ack_acceptance_is_not_confirmation(self):
        self.link.confirm = False
        result = self.action(action='mode', mode='LOITER')
        self.assertFalse(result['confirmed'])
        self.assertEqual(result['ack'], 0)

    def test_armed_mode_change_and_airborne_disarm_rejected(self):
        self.link.state.update(armed=True, landed_state=2)
        with self.assertRaises(ControlError):
            self.action(action='mode', mode='LOITER')
        with self.assertRaises(ControlError):
            self.action(action='disarm')
        self.assertEqual(self.link.commands, [])

    def test_watchdog_has_no_ten_second_bench_disarm(self):
        self.link.state.update(armed=True)
        self.console.bench_deadline = time.monotonic() - 1
        self.console.zero_enabled = False
        worker = threading.Thread(target=self.console._watchdog)
        worker.start()
        try:
            time.sleep(.5)
            self.assertTrue(self.link.state['armed'])
            self.assertEqual(self.link.commands, [])
            self.assertIsNone(self.console.status()['data']['control']['bench_remaining_s'])
        finally:
            self.console.stop_event.set()
            worker.join(2)

    def test_single_control_switch(self):
        with self.assertRaises(ControlError):
            self.console.claim('another-client')
        self.console.lease_until = 0
        with self.assertRaises(ControlError):
            self.action(action='mode', mode='LOITER')
        self.assertEqual(self.link.commands, [])

    def test_unexpected_error_is_not_replayed(self):
        def failure(number, params, timeout=3):
            self.link.commands.append((number, params))
            raise OSError('注入发送后断线')
        self.link.command = failure
        result = self.action(action='mode', mode='LOITER')
        self.assertFalse(result['ok'])
        self.assertEqual(self.action(action='mode', mode='LOITER'), result)
        self.assertEqual(len(self.link.commands), 1)

    def test_disable_disarms_then_exits_offboard(self):
        self.link.state.update(armed=True)
        result = self.action(action='disable')
        self.assertTrue(result['ok'])
        self.assertFalse(self.link.state['armed'])
        self.assertEqual(self.link.state['mode'], 'LOITER')
        self.assertFalse(self.console.zero_enabled)
        self.assertIsNone(self.console.owner)
        self.assertEqual(self.link.commands[0], (400, [0, 0, 0, 0, 0, 0, 0]))

    def test_disable_does_not_claim_success_when_disarm_denied(self):
        self.link.state.update(armed=True)
        self.link.ack = 2
        result = self.action(action='disable')
        self.assertFalse(result['confirmed'])
        self.assertIsNotNone(self.console.owner)
        self.assertTrue(self.link.state['armed'])

    def test_http_request_origin_and_whitelist(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.console = self.console
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = 'http://127.0.0.1:' + str(server.server_port)
        try:
            for route, headers, expected in (
                ('/api/control/action', {}, 403),
                ('/api/control/action', {'X-Control-Request': '1', 'Origin': 'http://evil.test'}, 403),
                ('/api/control/unknown', {'X-Control-Request': '1'}, 404)):
                request = Request(base + route, data=b'{}', headers={'Content-Type': 'application/json', **headers})
                with self.assertRaises(HTTPError) as caught:
                    urlopen(request, timeout=2)
                self.assertEqual(caught.exception.code, expected)
            with urlopen(base + '/api/status') as response:
                result = json.load(response)
                self.assertEqual(result['source'], 'px4')
            self.assertEqual(self.link.commands, [])
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)


if __name__ == '__main__':
    unittest.main()
