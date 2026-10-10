"""网页真实控制服务。复用 PX4Link，提供遥测、模式和有限运动任务。"""
import copy
import math
import threading
import time
from collections import OrderedDict

from link import PX4Link
from motion_tasks import MotionTasks
from joystick_control import JoystickControl

MODES = {'OFFBOARD': (6, 0), 'LOITER': (4, 3),
         'ALTCTL': (2, 0), 'POSCTL': (3, 0), 'STABILIZED': (7, 0), 'LAND': (4, 6)}
ACK_NAMES = {0: '接受', 1: '暂时拒绝', 2: '拒绝', 3: '不支持',
             4: '失败', 5: '处理中', 6: '取消'}


def clean(value):
    """MAVLink 未知值可能是 NaN；标准 JSON 必须用 null。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


class TelemetryLink(PX4Link):
    def _receive(self, msg, now):
        super()._receive(msg, now)
        with self.lock:
            if (msg.get_srcSystem(), msg.get_srcComponent()) != self.target:
                return
            kind = msg.get_type()
            if kind == 'HEARTBEAT' and msg.autopilot == 12:
                self.state['heartbeat_revision'] = self.state.get('heartbeat_revision', 0) + 1
            elif kind == 'SYS_STATUS':
                self.state.update(battery_at=now,
                    battery_current_a=None if msg.current_battery == -1 else msg.current_battery / 100,
                    battery_remaining_pct=None if msg.battery_remaining == -1 else msg.battery_remaining,
                    cpu_load_pct=msg.load / 10)
            elif kind == 'ATTITUDE':
                self.state.update(attitude_at=now, angular_rate_rad_s={
                    axis: getattr(msg, axis) for axis in ('rollspeed', 'pitchspeed', 'yawspeed')})
            elif kind == 'EXTENDED_SYS_STATE':
                self.state['landed_at'] = now
            elif kind == 'LOCAL_POSITION_NED':
                self.state['position_sample_ms'] = msg.time_boot_ms
                self.state['position_revision'] = self.state.get('position_revision', 0) + 1
            elif kind == 'ODOMETRY':
                self.state.update(odometry_at=now, odometry_reset_counter=getattr(msg, 'reset_counter', None))
            elif kind == 'DISTANCE_SENSOR' and msg.orientation == 25:
                self.state.update(range_m=msg.current_distance / 100, range_at=now)
            elif kind == 'OPTICAL_FLOW_RAD':
                self.state.update(flow_quality=msg.quality, flow_at=now)
            elif kind == 'RC_CHANNELS':
                self.state.update(rc_at=now, rc_rssi=None if msg.rssi == 255 else msg.rssi,
                                  rc_channel_count=msg.chancount)

    def status(self):
        state = super().status()
        now = time.monotonic()
        for key in ('position', 'attitude', 'battery', 'estimator', 'range', 'flow', 'rc', 'landed', 'odometry'):
            timestamp = state.pop(key + '_at', None)
            state[key + '_age_s'] = round(max(0, now - timestamp), 2) if timestamp else None
        return clean(state)

    def send_motion(self, vx, vy, vz, yaw_rate):
        """局部NED速度+m/s、偏航角速度rad/s；忽略位置/加速度/偏航角。"""
        with self.lock:
            if not self.connection or not self.target:
                raise RuntimeError('飞控未连接')
            self.connection.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xffffffff,
                *self.target, 1, 1479, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)


class ControlError(Exception):
    def __init__(self, message, code=409):
        super().__init__(message)
        self.code = code


class FlightConsole:
    LEASE_SECONDS = 20

    def __init__(self, link=None):
        self.link = link or TelemetryLink()
        self.lock = threading.RLock()
        self.action_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.owner = None
        self.lease_until = 0
        self.zero_enabled = False
        self.zero_started = 0
        self.zero_sent = 0
        self.zero_error = None
        self.bench_deadline = None
        self.last_action = None
        self.cache = OrderedDict()
        self.threads = []
        self.motion = MotionTasks()
        self.joystick = JoystickControl()
        self.flight_session = False
        self.close_after_land = False

    def start(self):
        self.link.start()
        for name, target in (('web-zero-stream', self._stream), ('web-watchdog', self._watchdog)):
            thread = threading.Thread(target=target, daemon=True, name=name)
            self.threads.append(thread)
            thread.start()

    def claim(self, client):
        if not isinstance(client, str) or not 8 <= len(client) <= 100:
            raise ControlError('客户端标识无效', 400)
        with self.lock:
            if self.owner != client and time.monotonic() < self.lease_until:
                raise ControlError('另一浏览器正在控制，请先释放控制权')
            self.owner = client
            self.lease_until = time.monotonic() + self.LEASE_SECONDS
        return {'ok': True, 'lease_seconds': self.LEASE_SECONDS}

    def _owned(self, client):
        if self.owner != client or time.monotonic() >= self.lease_until:
            raise ControlError('控制权已过期，请重新取得控制权', 403)

    def renew(self, client):
        with self.lock:
            self._owned(client)
            self.lease_until = time.monotonic() + self.LEASE_SECONDS
        return {'ok': True}

    def status(self):
        with self.lock:
            control = dict(scope='实机运动控制' if self.flight_session else '实机调试', lease_active=time.monotonic() < self.lease_until,
                zero_stream=self.zero_enabled, zero_error=self.zero_error,
                zero_age_s=round(time.monotonic() - self.zero_sent, 2) if self.zero_sent else None,
                bench_remaining_s=None,
                last_action=copy.deepcopy(self.last_action))
            motion = self.motion.status(time.monotonic())
            joystick = self.joystick.status(time.monotonic())
            control['source'] = ('joystick' if self.joystick.active else
                'px4_land' if motion.get('state') == 'landing' else
                'task' if motion.get('state') == 'running' else
                'hold' if self.motion.hold else 'zero' if self.zero_enabled else 'monitor')
        return {'source': 'px4', 'actuation_enabled': True,
                'data': {'px4': self.link.status(), 'control': control, 'motion': motion,
                         'joystick': joystick, 'vision': {}}}

    def stick_input(self, client, body):
        """连续输入独立于慢命令/回执队列；只更新内存，不在HTTP线程发MAVLink。"""
        with self.lock:
            self._owned(client)
            state = self._connected()
            if not self.zero_enabled or time.monotonic() - self.zero_sent > .3:
                raise ControlError('Offboard设定值流未就绪，摇杆输入不接收')
            error = self.joystick.ready_error(state)
            if error:
                self.joystick.stop(error, failed=True)
                raise ControlError(error)
            try:
                self.joystick.receive(body, time.monotonic())
            except ValueError as error:
                raise ControlError(str(error))
            return dict(ok=True, sequence=self.joystick.sequence,
                        joystick=self.joystick.status(time.monotonic()))

    def _stop_sticks(self, state, message, hold=False):
        self.joystick.stop(message)
        self.motion.clear_hold()
        if hold:
            self.motion.hold_current(state)

    def _connected(self):
        state = self.link.status()
        if not state.get('connected') or not state.get('bidirectional'):
            raise ControlError('需要新鲜心跳和双向参数回复；当前链路未就绪')
        return state

    @staticmethod
    def _fresh(state, key, limit):
        age = state.get(key + '_age_s')
        return isinstance(age, (float, int)) and 0 <= age < limit

    def _velocity_valid(self, state):
        flags = state.get('estimator_flags', 0)
        if not (self._fresh(state, 'position', 2) and self._fresh(state, 'estimator', 6)
                and flags & 6 == 6 and not flags & 128):
            raise ControlError('速度估计过期或无效，不能进入速度 Offboard；不会绕过飞控检查')

    def _confirmed_command(self, command, params, predicate):
        revision = self.link.status().get('heartbeat_revision', 0)
        ack = None
        error = None
        try:
            ack = self.link.command(command, params, timeout=3)
        except TimeoutError as exc:
            error = str(exc)
        deadline = time.monotonic() + 3
        state = self.link.status()
        confirmed = False
        while time.monotonic() < deadline:
            state = self.link.status()
            if state.get('connected') and state.get('heartbeat_revision', 0) > revision and predicate(state):
                confirmed = True
                break
            if ack not in (None, 0, 5):
                break
            self.stop_event.wait(.05)
        return dict(ok=confirmed and ack in (None, 0, 5), confirmed=confirmed,
            ack=ack, ack_text=ACK_NAMES.get(ack, '未收到回执'),
            message=('新心跳已确认目标状态' if confirmed else '目标状态未确认；请检查飞控状态，勿自动重发'),
            error=error, armed=state.get('armed'), mode=state.get('mode'),
            last_text=state.get('last_text'))

    def perform(self, client, body):
        request_id = body.get('request_id')
        if not isinstance(request_id, str) or not 8 <= len(request_id) <= 100:
            raise ControlError('需要唯一 request_id', 400)
        with self.lock:
            self._owned(client)
        if not self.action_lock.acquire(blocking=False):
            raise ControlError('上一操作尚在等待飞控反馈，请勿并发提交')
        try:
            with self.lock:
                cache_key = (client, request_id)
                if cache_key in self.cache:
                    previous_body, result = self.cache[cache_key]
                    if previous_body != body:
                        raise ControlError('同一请求编号不能用于不同操作', 400)
                    return copy.deepcopy(result)
            try:
                result = self._perform(body)
            except ControlError:
                raise
            except Exception as error:
                # 发送后链路异常时也缓存未知结果，重复HTTP请求不能再次解锁。
                result = dict(ok=False, confirmed=False, message='操作结果未知：' + str(error))
            result.update(request_id=request_id, action=body.get('action'))
            with self.lock:
                self.last_action = copy.deepcopy(result)
                self.cache[cache_key] = (copy.deepcopy(body), copy.deepcopy(result))
                while len(self.cache) > 128:
                    self.cache.popitem(last=False)
            return result
        finally:
            self.action_lock.release()

    def _perform(self, body):
        action = body.get('action')
        if action not in ('mode', 'arm', 'disarm', 'release', 'disable', 'task_start', 'task_cancel', 'takeoff', 'land', 'stick_start', 'stick_stop'):
            raise ControlError('未知实机控制操作', 400)
        if action == 'stick_stop':
            with self.lock:
                if body.get('session') != self.joystick.session:
                    raise ControlError('摇杆会话不匹配；旧页面不能结束新会话')
                if self.joystick.active:
                    self._stop_sticks(self.link.status(), '双摇杆已退出，停止原输入', hold=True)
            return dict(ok=True, confirmed=True, message='双摇杆已退出；有效反馈下保持退出时位置')
        if action in ('land', 'disable', 'disarm', 'release', 'mode', 'arm'):
            with self.lock:
                if self.joystick.active:
                    self._stop_sticks(self.link.status(), '摇杆已让出控制，执行' + action)
        state = self._connected()
        if action == 'stick_start':
            with self.lock:
                task = self.motion.task or {}
                if task.get('state') == 'landing':
                    raise ControlError('PX4正在执行降落，不能混入摇杆速度')
                if not self.zero_enabled or time.monotonic() - self.zero_sent > .3:
                    raise ControlError('先进入OFFBOARD，等待设定值流就绪')
                try:
                    joystick = self.joystick.start(state, time.monotonic())
                except ValueError as error:
                    raise ControlError(str(error))
                if task.get('state') == 'running':
                    self.motion._terminal('cancelled', '双摇杆已接管，原运动目标取消，不自动恢复')
                self.motion.clear_hold()
                self.bench_deadline = None
                self.flight_session = True
            return dict(ok=True, confirmed=True, session=joystick['session'],
                        message='双摇杆已接管；旧自动目标不再执行')
        if action in ('task_start', 'takeoff'):
            with self.lock:
                if self.joystick.active:
                    raise ControlError('先退出双摇杆，再开始自动运动任务')
                if not self.zero_enabled or time.monotonic() - self.zero_sent > .3:
                    raise ControlError('先进入OFFBOARD，等待设定值流就绪')
                raw_params = body.get('task', {})
                if not isinstance(raw_params, dict):
                    raise ControlError('任务参数需要JSON对象', 400)
                params = dict(raw_params)
                if action == 'takeoff':
                    params.update(kind='takeoff', direction='up')
                try:
                    task = self.motion.start(params, state, time.monotonic())
                except ValueError as error:
                    raise ControlError(str(error))
                self.bench_deadline = None
                self.flight_session = True
                self.close_after_land = False
            return dict(ok=True, accepted=True, confirmed=False, task_id=task['id'],
                        message='真实任务已启动，完成状态由遥测反馈判定')
        if action == 'task_cancel':
            with self.lock:
                if self.joystick.active:
                    raise ControlError('当前由双摇杆控制，请使用退出摇杆按钮')
                try:
                    self.motion.cancel(state, time.monotonic())
                except ValueError as error:
                    raise ControlError(str(error))
            return dict(ok=True, accepted=True, confirmed=False, message='已取消运动目标，开发板保持当前位置；不等同电机停转')
        if action == 'land':
            if state.get('armed') is False:
                if state.get('landed_state') == 1 and self._fresh(state, 'landed', 3):
                    return dict(ok=True, confirmed=True, message='飞控已落地并上锁，无需发送降落命令')
                raise ControlError('尚未确认落地，解锁状态为上锁；不提交降落命令')
            if state.get('armed') is not True:
                raise ControlError('解锁状态未知')
            with self.lock:
                self.motion.begin_land(time.monotonic())
                self.bench_deadline = None
                self.flight_session = True
                self.close_after_land = False
            result = self._confirmed_command(176, [1, 4, 6, 0, 0, 0, 0], lambda s: s.get('mode') == 'LAND')
            with self.lock:
                self.motion.land_reply(result['confirmed'])
                if result['confirmed']:
                    self.zero_enabled = False
            result.update(message='PX4 LAND模式已确认；等待真实落地和上锁反馈' if result['confirmed']
                          else 'LAND模式未确认，请检查飞控实际状态', accepted=result['confirmed'])
            return result
        if action == 'disable':
            if state.get('armed') is True:
                if state.get('landed_state') != 1 or not self._fresh(state, 'landed', 3):
                    result = self._perform({'action': 'land'})
                    with self.lock:
                        self.close_after_land = bool(result['confirmed'])
                    result.update(control_disabled=False, confirmed=False,
                                  message='正在尝试降落，落地上锁后关闭控制' if result['accepted'] else result['message'])
                    return result
                result = self._perform({'action': 'disarm'})
                if not result['confirmed']:
                    return result
                state = self._connected()
            if state.get('armed') is not False:
                raise ControlError('未确认上锁，关闭操作尚未完成')
            if state.get('mode') == 'OFFBOARD':
                result = self._perform({'action': 'release'})
                if not result['confirmed']:
                    return result
            with self.lock:
                self.owner = None
                self.lease_until = 0
                self.zero_enabled = False
                self.motion.clear_hold()
                self.flight_session = False
                self.close_after_land = False
            return dict(ok=True, confirmed=True, control_disabled=True, message='实机控制已关闭，已确认未解锁')
        if action == 'disarm':
            if state.get('armed') is False:
                return dict(ok=True, confirmed=True, message='飞控已经上锁，无需发送命令')
            if state.get('landed_state') != 1 or not self._fresh(state, 'landed', 3):
                raise ControlError('尚未确认在地面，请使用降落；不提供空中停机')
            result = self._confirmed_command(400, [0, 0, 0, 0, 0, 0, 0], lambda s: s.get('armed') is False)
            if result['confirmed']:
                with self.lock:
                    self.bench_deadline = None
            return result
        if action == 'arm':
            if state.get('armed') is not False:
                raise ControlError('飞控未处于已确认的上锁状态')
            if state.get('mode') != 'OFFBOARD':
                raise ControlError('先在未解锁状态切换 OFFBOARD，检查零速度数据流')
            self._velocity_valid(state)
            if state.get('landed_state') != 1 or not self._fresh(state, 'landed', 3):
                raise ControlError('需要新鲜的飞控在地面状态')
            if not self._fresh(state, 'battery', 3) or not isinstance(state.get('battery_v'), (int, float)) or state['battery_v'] < 14.4:
                raise ControlError('当前4S电池电压未知、陈旧或低于14.4V，请先检查供电')
            with self.lock:
                if not self.zero_enabled or time.monotonic() - self.zero_sent > .3:
                    raise ControlError('Offboard 零速度流未就绪')
            return self._confirmed_command(400, [1, 0, 0, 0, 0, 0, 0], lambda s: s.get('armed') is True)
        if state.get('armed') is not False:
            raise ControlError('当前接口只允许未解锁模式切换；请先完成地面上锁')
        mode = 'LOITER' if action == 'release' else body.get('mode')
        if mode not in MODES:
            raise ControlError('不支持此模式', 400)
        if mode == 'LAND':
            raise ControlError('降落请使用独立降落按钮', 400)
        if mode == 'OFFBOARD':
            self._velocity_valid(state)
            with self.lock:
                self.zero_enabled = True
                self.zero_error = None
                self.zero_started = 0
                self.zero_sent = 0
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                with self.lock:
                    ready = self.zero_enabled and self.zero_started and time.monotonic() - self.zero_started >= 1.5 and time.monotonic() - self.zero_sent < .3
                if ready:
                    break
                self.stop_event.wait(.05)
            if not ready:
                raise ControlError('零速度流预发送未完成，请检查链路')
            state = self._connected()
            if state.get('armed') is not False:
                raise ControlError('预发送期间解锁状态变化，已中止模式请求')
            self._velocity_valid(state)
        main, sub = MODES[mode]
        result = self._confirmed_command(176, [1, main, sub, 0, 0, 0, 0], lambda s: s.get('mode') == mode)
        if mode != 'OFFBOARD' and result['confirmed']:
            with self.lock:
                self.zero_enabled = False
                self.motion.clear_hold()
        if action == 'release' and result['confirmed']:
            with self.lock:
                self.owner = None
                self.lease_until = 0
        return result

    def _stream_once(self):
        """选择唯一控制来源并发送；持锁到写串口结束，切换不会漏发旧目标。"""
        state = self.link.status()
        with self.lock:
            enabled = self.zero_enabled
            previous = self.zero_sent
            manual = self.joystick.active
            if manual:
                target = self.joystick.update(state, time.monotonic(), time.monotonic() < self.lease_until)
                if not self.joystick.active:
                    # 输入停更即停止旧杆量；不恢复接管前的任务。
                    self.motion.clear_hold()
                    if state.get('odometry_reset_counter') == self.joystick.reset_counter:
                        self.motion.hold_current(state)
            else:
                target = self.motion.update(state, time.monotonic(), time.monotonic() < self.lease_until)
            if enabled:
                try:
                    if not state.get('connected'):
                        raise RuntimeError('心跳失联，零速度流已锁止；恢复连接后需重新操作')
                    if previous and time.monotonic() - previous > .3:
                        raise RuntimeError('零速度发送间隔超过0.3秒，已锁止')
                    if target is not None:
                        if manual or self.motion.task or self.motion.hold:
                            self.link.send_motion(*target)
                        else:
                            self.link.send_velocity(0, 0, 0)
                    self.zero_sent = time.monotonic()
                    if not self.zero_started:
                        self.zero_started = self.zero_sent
                except Exception as error:
                    self.zero_enabled = False
                    self.zero_error = str(error)
                    self.joystick.stop('设定值发送失败，摇杆已停止', failed=True)
                    self.motion._terminal('failed', '设定值发送失败：' + str(error))

    def _stream(self):
        while not self.stop_event.is_set():
            self._stream_once()
            self.stop_event.wait(.05)

    def _watchdog(self):
        while not self.stop_event.wait(.2):
            state = self.link.status()
            if not state.get('connected'):
                continue
            with self.lock:
                if self.zero_enabled and self.zero_sent and time.monotonic() - self.zero_sent > .3:
                    self.zero_enabled = False
                    self.zero_error = '设定值线程未按时更新，已停止原任务'
                    self.joystick.stop(self.zero_error, failed=True)
                    self.motion._terminal('failed', self.zero_error)
                expired_owner = time.monotonic() >= self.lease_until and self.zero_enabled
                task = self.motion.status(time.monotonic())
                landing_disarm = task.get('state') == 'landing' and task.get('needs_disarm')
                close_landed = self.close_after_land and task.get('state') == 'succeeded' and state.get('armed') is False
                if close_landed:
                    self.close_after_land = False
                    self.owner = None
                    self.lease_until = 0
                    self.flight_session = False
            if not (expired_owner or landing_disarm) or not self.action_lock.acquire(blocking=False):
                continue
            try:
                if landing_disarm:
                    result = self._perform({'action': 'disarm'})
                    result['action'] = '降落后普通上锁'
                elif state.get('armed') is False:
                    result = self._perform({'action': 'release'})
                    result['action'] = '控制权超时：地面退出Offboard'
                else:
                    continue
                with self.lock:
                    self.last_action = result
            except Exception as error:
                with self.lock:
                    self.last_action = dict(ok=False, message=str(error), action='控制状态收尾')
                self.stop_event.wait(1)
            finally:
                self.action_lock.release()

    def stop(self):
        # 先在发送线程仍运行时尝试普通地面收尾；不发送强制解锁/上锁。
        with self.action_lock:
            state = self.link.status()
            try:
                if self.flight_session and state.get('connected') and state.get('armed') and state.get('landed_state') != 1:
                    self._perform({'action': 'land'})
                if state.get('connected') and state.get('armed') and state.get('landed_state') == 1 and self._fresh(state, 'landed', 3):
                    self._perform({'action': 'disarm'})
                state = self.link.status()
                if self.zero_enabled and state.get('connected') and state.get('armed') is False:
                    self._perform({'action': 'release'})
            except Exception as error:
                print('关闭时未确认收尾：', error, flush=True)
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1)
        self.link.stop()
