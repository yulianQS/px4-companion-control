"""基于真实遥测的有限运动任务；纯计算模块，不直接访问串口。"""
import copy
import math
import uuid

ACTIVE = ('running', 'landing')
ZERO = (0., 0., 0., 0.)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def limited(value, bound):
    return max(-bound, min(bound, value))


def recent(state, name, seconds):
    age = state.get(name + '_age_s')
    return isinstance(age, (int, float)) and 0 <= age < seconds


def pose(state):
    position = state.get('local_position') or {}
    attitude = state.get('attitude_deg') or []
    values = [position.get(key) for key in ('x', 'y', 'z', 'vx', 'vy', 'vz')]
    if len(attitude) != 3:
        raise ValueError('缺少姿态反馈')
    values += [attitude[2]]
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
        raise ValueError('位置、速度或偏航反馈未知')
    return values[:6] + [math.radians(values[6])]


def feedback_error(state):
    if not state.get('connected'):
        return '飞控心跳失联'
    if not recent(state, 'position', 1) or not recent(state, 'attitude', .5):
        return '位置或姿态反馈过期'
    if state.get('odometry_reset_counter') is None or not recent(state, 'odometry', 1):
        return '估计器重置信息缺失或过期'
    flags = state.get('estimator_flags', 0)
    if not isinstance(flags, int):
        return '估计器有效标志未知'
    # MAVLink：2/4为水平/垂直速度，8为相对水平位置，32为局部垂直位置。
    if not recent(state, 'estimator', 6) or flags & 46 != 46 or flags & 128:
        return '局部位置/速度估计无效'
    try:
        pose(state)
    except ValueError as error:
        return str(error)
    rate = (state.get('angular_rate_rad_s') or {}).get('yawspeed')
    if not isinstance(rate, (int, float)) or not math.isfinite(rate):
        return '偏航角速度反馈未知'
    return None


class MotionTasks:
    """由唯一20Hz发送线程调用update；外层FlightConsole锁负责并发保护。"""
    def __init__(self):
        self.task = None
        self.hold = None
        self.last_command = ZERO
        self.last_tick = None
        self.reset_counter = None
        self.last_sample = None
        self.last_yaw = None
        self.yaw_travel = 0.
        self.settled_at = None
        self.settled_samples = 0
        self.last_settled_sample = None
        self.updated_at = None

    @staticmethod
    def parameters(body):
        kind = body.get('kind')
        if kind not in ('move', 'height', 'rotate', 'takeoff'):
            raise ValueError('未知运动类型')
        directions = {'move': ('forward', 'back', 'left', 'right'),
            'height': ('up', 'down'), 'rotate': ('clockwise', 'counterclockwise'),
            'takeoff': ('up',)}
        direction = body.get('direction', 'up' if kind == 'takeoff' else None)
        if direction not in directions[kind]:
            raise ValueError('运动方向无效')
        result = {'kind': kind, 'direction': direction}
        bounds = {'distance': (1, 360) if kind == 'rotate' else (.1, 5 if kind == 'move' else 2),
                  'speed': (5, 60) if kind == 'rotate' else (.05, .5 if kind == 'move' else .3),
                  'tolerance': (1, 15) if kind == 'rotate' else (.01, .5),
                  'timeout': (2, 120)}
        defaults = {'distance': .5, 'speed': .2, 'tolerance': .05, 'timeout': 30}
        for key, (low, high) in bounds.items():
            value = body.get(key, defaults[key] if kind == 'takeoff' else None)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('%s需要在%s～%s范围内' % (key, low, high))
            result[key] = float(value)
        if result['tolerance'] >= result['distance']:
            raise ValueError('允许误差必须小于目标距离或转角')
        return result

    def start(self, body, state, now):
        if self.task and self.task['state'] in ACTIVE:
            raise ValueError('已有任务运行，请先取消或降落')
        params = self.parameters(body)
        error = feedback_error(state)
        if error:
            raise ValueError(error)
        if state.get('armed') is not True or state.get('mode') != 'OFFBOARD':
            raise ValueError('先启用控制、进入OFFBOARD并普通解锁；任务不会自动解锁')
        if not recent(state, 'landed', 3):
            raise ValueError('飞控地面/空中状态未知')
        if params['kind'] == 'takeoff':
            if state.get('landed_state') != 1:
                raise ValueError('起飞任务需要飞控确认在地面')
        elif state.get('landed_state') != 2:
            raise ValueError('平移、转向和升降需要先起飞并确认在空中')
        current = pose(state)
        start = current[:3]
        yaw = current[6]
        goal = list(start)
        distance = params['distance']
        kind, direction = params['kind'], params['direction']
        if kind == 'move':
            forward, right = {'forward': (1, 0), 'back': (-1, 0),
                              'right': (0, 1), 'left': (0, -1)}[direction]
            goal[0] += distance * (math.cos(yaw) * forward - math.sin(yaw) * right)
            goal[1] += distance * (math.sin(yaw) * forward + math.cos(yaw) * right)
        elif kind in ('height', 'takeoff'):
            goal[2] += distance * (-1 if direction == 'up' else 1)
        self.task = dict(id=uuid.uuid4().hex, state='running', message='真实任务已接收，等待遥测反馈',
            params=params, started_at=now, elapsed_s=0., progress=0., measured=0., error=distance,
            start_position=start, goal_position=goal, start_yaw_deg=math.degrees(yaw),
            relative_yaw_deg=0., actual_position=list(start), feedback_fresh=True,
            target_reached=False)
        self.hold = dict(position=goal, yaw=yaw)
        self.reset_counter = state.get('odometry_reset_counter')
        self.last_sample = state.get('position_sample_ms')
        self.last_yaw = yaw
        self.yaw_travel = 0.
        self.last_command = ZERO
        self.last_tick = now
        self.updated_at = now
        self._reset_settled()
        return self.status(now)

    def _reset_settled(self):
        self.settled_at = None
        self.settled_samples = 0
        self.last_settled_sample = None

    def _terminal(self, status, message, state=None):
        if self.task:
            self.task.update(state=status, message=message)
        self.last_command = ZERO
        if state is not None and not feedback_error(state):
            current = pose(state)
            self.hold = dict(position=current[:3], yaw=current[6])
            self.reset_counter = state.get('odometry_reset_counter')
        else:
            self.hold = None

    def cancel(self, state, now):
        if self.task and self.task['state'] == 'landing':
            raise ValueError('PX4正在执行降落，取消不能自动恢复悬停')
        self._terminal('cancelled', '任务已取消，发送零速度并保持当前局部位置', state)
        return self.status(now)

    def clear_hold(self):
        self.hold = None
        self.last_command = ZERO

    def hold_current(self, state):
        """手动控制退出时捕获新位置，不能恢复接管前的自动目标。"""
        self.clear_hold()
        if feedback_error(state) or state.get('armed') is not True or state.get('mode') != 'OFFBOARD':
            return
        current = pose(state)
        self.hold = dict(position=current[:3], yaw=current[6])
        self.reset_counter = state.get('odometry_reset_counter')
        self.last_sample = state.get('position_sample_ms')
        self.last_yaw = current[6]
        self.yaw_travel = 0.
        self.last_tick = None

    def begin_land(self, now):
        self.task = dict(id=uuid.uuid4().hex, state='landing', message='请求PX4 LAND模式，尚未确认落地',
            params={'kind': 'land', 'timeout': 60}, started_at=now, elapsed_s=0.,
            progress=None, measured=None, error=None, land_mode_confirmed=False,
            needs_disarm=False, target_reached=False)
        self.clear_hold()
        self._reset_settled()

    def land_reply(self, confirmed):
        if self.task and self.task['state'] == 'landing':
            self.task['land_mode_confirmed'] = confirmed
            if not confirmed:
                self._terminal('failed', 'LAND模式未确认；检查飞控实际模式，不自动重发')

    def status(self, now):
        if not self.task:
            return {'state': 'idle', 'message': '尚未提交实机运动任务'}
        result = copy.deepcopy(self.task)
        result.pop('started_at', None)
        result['holding_position'] = self.hold is not None
        result['command_ned'] = list(self.last_command)
        result['feedback_age_s'] = round(max(0, now - self.updated_at), 2) if self.updated_at is not None else None
        return result

    def _landing_update(self, state, now):
        task = self.task
        if not state.get('connected'):
            task.update(message='心跳失联，降落结果未知', feedback_fresh=False)
        elif task['land_mode_confirmed']:
            if state.get('mode') != 'LAND' and state.get('armed') is True:
                self._terminal('failed', '飞控已退出LAND模式，降落任务停止监测')
            elif recent(state, 'landed', 3) and state.get('landed_state') == 1:
                if self.settled_at is None:
                    self.settled_at = now
                if now - self.settled_at >= 1:
                    if state.get('armed') is False:
                        self._terminal('succeeded', '飞控确认落地且已上锁')
                        task['target_reached'] = True
                    else:
                        task.update(needs_disarm=True, message='飞控确认落地，等待普通上锁确认')
            else:
                self.settled_at = None
                task.update(needs_disarm=False, message='PX4 LAND已确认，等待真实落地状态')
        if task['state'] == 'landing' and task['elapsed_s'] >= task['params']['timeout']:
            self._terminal('timed_out', '未在60秒内确认落地/上锁；PX4 LAND可能仍在继续，请观察真实状态')
        return ZERO if not task.get('land_mode_confirmed') and state.get('mode') == 'OFFBOARD' else None

    def update(self, state, now, owner_active=True):
        self.updated_at = now
        task = self.task
        if task and task['state'] in ACTIVE:
            task['elapsed_s'] = round(max(0, now - task['started_at']), 2)
        if task and task['state'] == 'landing':
            return self._landing_update(state, now)
        running = bool(task and task['state'] == 'running')
        if not running and self.hold is None:
            return ZERO
        error = feedback_error(state)
        if not owner_active:
            error = '控制会话中断，任务锁止，不自动重放'
        if state.get('armed') is not True or state.get('mode') != 'OFFBOARD':
            error = '飞控已上锁或退出OFFBOARD，任务停止'
        if error:
            if running:
                task['feedback_fresh'] = False
                self._terminal('failed', error)
            self.clear_hold()
            return ZERO
        current = pose(state)
        reset = state.get('odometry_reset_counter')
        sample = state.get('position_sample_ms')
        if (self.reset_counter is not None and reset is not None and reset != self.reset_counter) or (
                self.last_sample is not None and sample is not None and sample < self.last_sample):
            if running:
                self._terminal('failed', '检测到估计器重置或飞控时间回退，已中止原坐标任务')
            self.clear_hold()
            return ZERO
        self.last_sample = sample
        yaw = current[6]
        delta = wrap(yaw - self.last_yaw) if self.last_yaw is not None else 0.
        if abs(delta) > math.radians(30):
            if running:
                self._terminal('failed', '偏航反馈发生不连续跳变，任务停止')
            self.clear_hold()
            return ZERO
        self.last_yaw = yaw
        self.yaw_travel += delta
        dt = min(.1, max(.001, now - self.last_tick)) if self.last_tick is not None else .05
        self.last_tick = now
        horizontal_limit, vertical_limit, yaw_limit = .2, .2, math.radians(30)
        goal = self.hold['position']
        error_xyz = [goal[index] - current[index] for index in range(3)]
        yaw_error = wrap(self.hold['yaw'] - yaw)
        if running:
            params = task['params']
            kind, direction = params['kind'], params['direction']
            task.update(actual_position=current[:3], relative_yaw_deg=round(math.degrees(self.yaw_travel), 2), feedback_fresh=True)
            if kind == 'rotate':
                sign = 1 if direction == 'clockwise' else -1
                measured = math.degrees(self.yaw_travel) * sign
                yaw_error = math.radians(params['distance'] * sign) - self.yaw_travel
                distance_error = abs(math.degrees(yaw_error))
                yaw_limit = math.radians(params['speed'])
            elif kind == 'move':
                displacement = [current[index] - task['start_position'][index] for index in range(2)]
                path = [goal[index] - task['start_position'][index] for index in range(2)]
                measured = sum(a * b for a, b in zip(displacement, path)) / params['distance']
                distance_error = math.hypot(*error_xyz[:2])
                horizontal_limit = params['speed']
            else:
                measured = (current[2] - task['start_position'][2]) * (-1 if direction == 'up' else 1)
                distance_error = abs(error_xyz[2])
                vertical_limit = params['speed']
            task.update(measured=round(measured, 3), error=round(distance_error, 3),
                        progress=round(max(0, min(1, measured / params['distance'])), 3))
            speed = math.sqrt(sum(value * value for value in current[3:6]))
            yaw_speed = abs((state.get('angular_rate_rad_s') or {}).get('yawspeed', math.inf))
            reached = (distance_error <= params['tolerance'] and speed < .08
                and abs(error_xyz[2]) <= (params['tolerance'] if kind in ('height', 'takeoff') else .1)
                and math.hypot(*error_xyz[:2]) <= (params['tolerance'] if kind == 'move' else .15)
                and (yaw_speed < math.radians(5) if kind == 'rotate' else abs(yaw_error) < math.radians(5)))
            if kind == 'takeoff':
                reached = reached and state.get('landed_state') == 2 and recent(state, 'landed', 3)
            elif state.get('landed_state') != 2 or not recent(state, 'landed', 3):
                self._terminal('failed', '飞控不再确认在空中，任务已停止', state)
                return ZERO
            if reached:
                if self.settled_at is None:
                    self.settled_at = now
                revision = state.get('position_revision')
                if revision is not None and revision != self.last_settled_sample:
                    self.settled_samples += 1
                    self.last_settled_sample = revision
                if now - self.settled_at >= .5 and self.settled_samples >= 3:
                    task.update(state='succeeded', message='真实反馈到达目标并稳定，继续保持目标位置', target_reached=True)
                    if kind == 'rotate':
                        self.hold['yaw'] = yaw
                    running = False
            else:
                self._reset_settled()
            if task['state'] == 'running' and task['elapsed_s'] >= params['timeout']:
                self._terminal('timed_out', '任务超时，停止原目标并保持当前位置', state)
                return ZERO
        desired = [.8 * error_xyz[0], .8 * error_xyz[1],
                   limited(error_xyz[2], vertical_limit), limited(1.5 * yaw_error, yaw_limit)]
        magnitude = math.hypot(*desired[:2])
        if magnitude > horizontal_limit:
            desired[0] *= horizontal_limit / magnitude
            desired[1] *= horizontal_limit / magnitude
        # 限加速度，避免任务开始/目标变化时直接阶跃到最大速度。
        delta_xy = [desired[index] - self.last_command[index] for index in range(2)]
        delta_norm = math.hypot(*delta_xy)
        if delta_norm > .3 * dt:
            for index in range(2):
                desired[index] = self.last_command[index] + delta_xy[index] * .3 * dt / delta_norm
        desired[2] = self.last_command[2] + limited(desired[2] - self.last_command[2], .3 * dt)
        desired[3] = self.last_command[3] + limited(desired[3] - self.last_command[3], math.radians(60) * dt)
        self.last_command = tuple(desired)
        return self.last_command
