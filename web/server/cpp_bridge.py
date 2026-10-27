"""TCP transport for the existing Python web GUI. No MAVLink or control loops."""
import copy
import ipaddress
import json
import socket
import struct
import threading
import time
import uuid
from .config import RUNTIME

MAX_FRAME = 1048576


class ControlError(RuntimeError):
    def __init__(self, message, code=409):
        super().__init__(message)
        self.code = code


def clean(value):
    import math
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return value


class ServiceClient:
    """One reader worker, serialized writes, per-command waiters; no UI mutations."""
    def __init__(self, host=None, port=None):
        host, port = host or RUNTIME['control']['host'], port or RUNTIME['control']['port']
        self.host, self.port = host, port
        self.lock = threading.RLock()
        self.send_lock = threading.Lock()
        self.socket = None
        self.pending = {}
        self.snapshot = None
        self.received_at = 0
        self.error = '等待C++控制服务'
        self.enabled = True
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True, name='cpp-telemetry')

    def start(self):
        self.worker.start()

    def _close(self):
        with self.lock:
            sock, self.socket = self.socket, None
            self.received_at = 0
            for waiter in self.pending.values():
                waiter['error'] = 'TCP连接中断，命令结果未知；不会自动重发'
                waiter['event'].set()
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def configure(self, host, port, enabled=True):
        try:
            ipaddress.IPv4Address(host)
            if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
                raise ValueError()
        except (ValueError, TypeError):
            raise ControlError('需要有效IPv4地址和1～65535整数端口', 400)
        with self.lock:
            self.host, self.port, self.enabled = host, port, bool(enabled)
        self._close()
        return {'ok': True, 'confirmed': False, 'message': '连接配置已更新，等待C++状态回传'}

    def _run(self):
        while not self.stop_event.is_set():
            with self.lock:
                enabled, endpoint = self.enabled, (self.host, self.port)
            if not enabled:
                self.stop_event.wait(.2)
                continue
            try:
                sock = socket.create_connection(endpoint, timeout=2)
                sock.settimeout(.5)
                with self.lock:
                    if endpoint != (self.host, self.port) or not self.enabled:
                        sock.close()
                        continue
                    self.socket = sock
                buffer = bytearray()
                last_data = time.monotonic()
                while not self.stop_event.is_set():
                    with self.lock:
                        if self.socket is not sock:
                            break
                    try:
                        block = sock.recv(65536)
                    except socket.timeout:
                        if time.monotonic() - last_data > 3:
                            raise TimeoutError('C++状态流超时')
                        continue
                    if not block:
                        raise ConnectionError('C++服务关闭连接')
                    last_data = time.monotonic()
                    buffer.extend(block)
                    while len(buffer) >= 4:
                        size = struct.unpack_from('!I', buffer)[0]
                        if not 0 < size <= MAX_FRAME:
                            raise ValueError('C++帧长度无效')
                        if len(buffer) < 4 + size:
                            break
                        message = json.loads(buffer[4:4 + size])
                        del buffer[:4 + size]
                        if not isinstance(message, dict):
                            raise ValueError('C++帧必须为JSON对象')
                        with self.lock:
                            if message.get('type') == 'status':
                                if message.get('source') != 'px4' or not isinstance(message.get('data'), dict):
                                    raise ValueError('C++状态格式无效')
                                self.snapshot = message
                                self.received_at = time.monotonic()
                                self.error = None
                            elif message.get('type') == 'response':
                                waiter = self.pending.get(str(message.get('id')))
                                if waiter and message.get('stage') != 'received':
                                    waiter['result'] = message
                                    waiter['event'].set()
            except (OSError, ValueError, TypeError) as error:
                with self.lock:
                    self.error = str(error)
            finally:
                self._close()
            self.stop_event.wait(.5)

    def request(self, command, params, client, request_id=None, timeout=10):
        request_id = request_id or uuid.uuid4().hex
        waiter = {'event': threading.Event()}
        body = json.dumps({'id': request_id, 'type': 'command', 'command': command,
                           'client': client, 'params': params}, allow_nan=False).encode('utf-8')
        if len(body) > MAX_FRAME:
            raise ControlError('指令过大', 413)
        with self.send_lock:
            with self.lock:
                sock = self.socket
                if sock is None or time.monotonic() - self.received_at > 1:
                    raise ControlError('C++控制服务未连接或状态过期', 503)
                if request_id in self.pending:
                    raise ControlError('同一请求正在执行')
                self.pending[request_id] = waiter
            try:
                sock.sendall(struct.pack('!I', len(body)) + body)
            except OSError as error:
                with self.lock:
                    self.pending.pop(request_id, None)
                raise ControlError('发送中断，结果未知：' + str(error), 503)
        try:
            if not waiter['event'].wait(timeout):
                raise ControlError('C++响应超时，执行结果未知，不会自动重发', 504)
            if 'error' in waiter:
                raise ControlError(waiter['error'], 503)
            return waiter['result']
        finally:
            with self.lock:
                self.pending.pop(request_id, None)

    def status(self):
        with self.lock:
            connected = self.socket is not None and time.monotonic() - self.received_at < 1
            result = copy.deepcopy(self.snapshot) if connected and self.snapshot else {
                'source': 'px4', 'actuation_enabled': False,
                'data': {'px4': {'connected': False, 'bidirectional': False,
                                 'armed': None, 'mode': None, 'message': self.error or 'C++未连接'},
                         'control': {}, 'motion': {}, 'joystick': {}, 'vision': {}, 'trajectory': {}}}
            result['data']['service'] = dict(connected=connected, host=self.host, port=self.port,
                                              enabled=self.enabled, error=self.error)
            return result

    def stop(self):
        self.stop_event.set()
        self._close()
        if self.worker.ident:
            self.worker.join(timeout=3)


class FlightConsole:
    ACTIONS = dict(mode='SET_MODE', arm='ARM', disarm='DISARM', release='RELEASE',
                   disable='DISABLE', task_start='MOVE', task_cancel='STOP_TRAJECTORY',
                   takeoff='TAKEOFF', land='LAND', stick_start='STICK_START', stick_stop='STICK_STOP')

    def __init__(self, host=None, port=None):
        self.client = ServiceClient(host, port)

    def start(self):
        self.client.start()

    def stop(self):
        self.client.stop()

    def status(self):
        return self.client.status()

    def claim(self, client):
        result = self.client.request('CLAIM', {}, client)
        if not result.get('ok'):
            raise ControlError(result.get('message', '控制权申请失败'))
        return result

    def renew(self, client):
        result = self.client.request('RENEW', {}, client)
        if not result.get('ok'):
            raise ControlError(result.get('message', '续约失败'))
        return result

    def perform(self, client, body):
        request_id = body.get('request_id')
        if not isinstance(request_id, str) or not 8 <= len(request_id) <= 100:
            raise ControlError('需要唯一request_id', 400)
        action = body.get('action')
        command = self.ACTIONS.get(action) if action != 'command' else body.get('command')
        allowed = set(self.ACTIONS.values()) | {'HOLD', 'SET_POSITION', 'SET_VELOCITY', 'EXECUTE_TRAJECTORY', 'VISION_START', 'VISION_STOP', 'VISION_UPDATE', 'GET_STATUS'}
        if command not in allowed:
            raise ControlError('未知控制命令', 400)
        params = body.get('params', {}) if action == 'command' else body.get('task', {})
        if not isinstance(params, dict):
            raise ControlError('参数必须是JSON对象', 400)
        params = dict(params)
        if action == 'mode':
            params['mode'] = body.get('mode')
        if action == 'stick_stop':
            params['session'] = body.get('session')
        result = self.client.request(command, params, client, request_id)
        result.update(request_id=request_id, action=action)
        return result

    def stick_input(self, client, body):
        result = self.client.request('STICK_INPUT', body, client)
        if not result.get('ok'):
            raise ControlError(result.get('message', '摇杆输入拒绝'))
        return result
