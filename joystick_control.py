"""双摇杆输入与速度映射；纯计算，不访问串口或浏览器。"""
import copy
import math
import uuid

from motion_tasks import ZERO, recent, limited


class JoystickControl:
    # 左杆：上升/顺时针；右杆：向前/向右。单位为m/s和度/s。
    HORIZONTAL_SPEED = .2
    VERTICAL_SPEED = .2
    YAW_RATE_DEG = 30.
    INPUT_TIMEOUT = .5
    DEADZONE = .08

    def __init__(self):
        self.active = False
        self.session = None
        self.sequence = -1
        self.received_at = None
        self.last_tick = None
        self.last_command = ZERO
        self.axes = dict(throttle=0., yaw=0., pitch=0., roll=0.)
        self.reset_counter = None
        self.state = 'idle'
        self.message = '进入OFFBOARD后可启用摇杆，支持未解锁地面调试'

    @staticmethod
    def ready_error(state):
        if not state.get('connected'):
            return '飞控心跳未连接'
        if state.get('mode') != 'OFFBOARD':
            return '先进入OFFBOARD，再启用摇杆'
        attitude = state.get('attitude_deg') or []
        if (not recent(state, 'attitude', .5) or len(attitude) != 3 or
                not isinstance(attitude[2], (int, float)) or not math.isfinite(attitude[2])):
            return '缺少新鲜偏航数据，无法转换机头方向速度'
        return None

    def start(self, state, now):
        if self.active:
            raise ValueError('双摇杆已经接管，请勿重复开启')
        error = self.ready_error(state)
        if error:
            raise ValueError(error)
        self.active = True
        self.session = uuid.uuid4().hex
        self.sequence = -1
        self.received_at = now
        self.last_tick = now
        self.last_command = ZERO
        self.axes = dict(throttle=0., yaw=0., pitch=0., roll=0.)
        self.reset_counter = state.get('odometry_reset_counter')
        self.state = 'waiting'
        self.message = '双摇杆已接管，等待中立输入'
        return self.status(now)

    def receive(self, body, now):
        if not self.active or body.get('session') != self.session:
            raise ValueError('摇杆会话已结束，请重新启用；旧输入不会恢复执行')
        # 收包线程也检查截止时间，不能用迟到的请求复活已过期会话。
        if now - self.received_at >= self.INPUT_TIMEOUT:
            self.stop('摇杆输入超过0.5秒未更新，已停止旧输入', failed=True)
            raise ValueError(self.message)
        sequence = body.get('sequence')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 2**53 - 1:
            raise ValueError('摇杆序号需要非负整数')
        if sequence <= self.sequence:
            raise ValueError('摇杆输入重复或乱序，已丢弃')
        axes = body.get('axes')
        if not isinstance(axes, dict) or set(axes) != set(self.axes):
            raise ValueError('需要throttle、yaw、pitch、roll四个输入')
        if not all(not isinstance(value, bool) and isinstance(value, (int, float))
                   and math.isfinite(value) and -1 <= value <= 1 for value in axes.values()):
            raise ValueError('摇杆输入必须为-1～1的有限数值')
        if self.sequence == -1 and any(abs(value) > self.DEADZONE for value in axes.values()):
            raise ValueError('启用后第一帧必须回中')
        self.axes = {name: float(value) for name, value in axes.items()}
        self.sequence = sequence
        self.received_at = now
        self.state = 'active'
        self.message = '双摇杆实时控制；升降杆松手保持，其余轴回中'

    def stop(self, message='双摇杆已退出', failed=False):
        self.active = False
        self.state = 'failed' if failed else 'stopped'
        self.message = message
        self.axes = dict(throttle=0., yaw=0., pitch=0., roll=0.)
        self.last_command = ZERO

    @classmethod
    def _pair(cls, x, y):
        radius = math.hypot(x, y)
        if radius <= cls.DEADZONE:
            return 0., 0.
        strength = (min(1., radius) - cls.DEADZONE) / (1. - cls.DEADZONE)
        return x / radius * strength, y / radius * strength

    def update(self, state, now, owner_active=True):
        if not self.active:
            return ZERO
        error = self.ready_error(state)
        if not owner_active:
            error = '控制会话已过期，摇杆锁止'
        if now - self.received_at >= self.INPUT_TIMEOUT:
            error = '摇杆输入超过0.5秒未更新，已停止旧输入'
        if error:
            self.stop(error, failed=True)
            return ZERO
        # 速度任务不依赖固定位置原点，不因位置重置而锁止；仅用当前偏航转换方向。
        yaw = math.radians(state['attitude_deg'][2])
        right, forward = self._pair(self.axes['roll'], self.axes['pitch'])
        # 升降杆保持独立杆量，偏航回中不应改变其对应的垂直速度。
        turn = self._pair(self.axes['yaw'], 0.)[0]
        up = self._pair(self.axes['throttle'], 0.)[0]
        desired = [self.HORIZONTAL_SPEED * (math.cos(yaw) * forward - math.sin(yaw) * right),
                   self.HORIZONTAL_SPEED * (math.sin(yaw) * forward + math.cos(yaw) * right),
                   -self.VERTICAL_SPEED * up, math.radians(self.YAW_RATE_DEG) * turn]
        # 回中优先停止旧速度，不继续执行上一杆量；非零目标限制加速度。
        if not any(desired):
            self.last_command = ZERO
        else:
            dt = min(.1, max(.001, now - self.last_tick))
            delta = [desired[i] - self.last_command[i] for i in range(2)]
            magnitude = math.hypot(*delta)
            if magnitude > .3 * dt:
                for i in range(2):
                    desired[i] = self.last_command[i] + delta[i] * .3 * dt / magnitude
            desired[2] = self.last_command[2] + limited(desired[2] - self.last_command[2], .3 * dt)
            desired[3] = self.last_command[3] + limited(desired[3] - self.last_command[3], math.radians(60) * dt)
            # 左右杆独立松开时，各自对应轴立即归零。
            if not (right or forward):
                desired[0:2] = [0., 0.]
            if not up:
                desired[2] = 0.
            if not turn:
                desired[3] = 0.
            self.last_command = tuple(desired)
        self.last_tick = now
        return self.last_command

    def status(self, now):
        return dict(active=self.active, state=self.state, session=self.session,
            message=self.message, axes=copy.deepcopy(self.axes), sequence=self.sequence,
            input_age_s=round(max(0, now - self.received_at), 2) if self.received_at is not None else None,
            command_ned=list(self.last_command), limits=dict(horizontal=self.HORIZONTAL_SPEED,
                vertical=self.VERTICAL_SPEED, yaw_deg=self.YAW_RATE_DEG, timeout_s=self.INPUT_TIMEOUT))
