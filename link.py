"""UART7 的唯一收发入口：接收状态，串行处理请求和匹配回执。"""
import copy
import math
import os
import threading
import time
from collections import Counter

# 本工程入口先导入 link，再导入 pymavlink；显式选择 MAVLink 2。
# 只影响当前 Python 进程，不修改飞控保存的参数或系统环境。
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
import settings


class PX4Link:
    def __init__(self, device=None):
        self.device = device or settings.UART_DEVICE
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.changed = threading.Condition(self.lock)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="mavlink")
        self.connection = None
        self.target = None
        self.last_ack = {}
        self.counts = Counter()
        self.state = {"message": "等待 UART7", "heartbeat_at": 0,
                      "bidirectional": False, "armed": None, "mode": None,
                      "last_text": None, "parameter_reply": None}
        self.probe_at = 0
        self.parameter_at = 0

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.ident:
            self.thread.join(timeout=3)

    def status(self):
        with self.lock:
            result = copy.deepcopy(self.state)
            result["counts"] = dict(self.counts)
            result["target"] = self.target
        age = time.monotonic() - result.pop("heartbeat_at")
        result["heartbeat_age_s"] = round(age, 2) if age < 1e6 else None
        result["connected"] = age < settings.HEARTBEAT_MAX_AGE
        result["bidirectional"] = (result["connected"] and result["bidirectional"]
                                   and time.monotonic() - self.parameter_at < 30)
        return result

    def _run(self):
        while not self.stop_event.is_set():
            try:
                connection = mavutil.mavlink_connection(
                    self.device, baud=settings.UART_BAUD,
                    source_system=settings.SOURCE_SYSTEM,
                    source_component=settings.SOURCE_COMPONENT,
                )
                try:
                    connection.port.exclusive = True
                except Exception:
                    connection.close()
                    raise
                with self.lock:
                    self.connection = connection
                    self.target = None
                    self.state.update(bidirectional=False, heartbeat_at=0,
                                      message="串口打开，等待 PX4 心跳")
                self.probe_at = 0
                self.parameter_at = 0
                while not self.stop_event.is_set():
                    msg = connection.recv_match(blocking=True, timeout=0.1)
                    now = time.monotonic()
                    if msg is not None:
                        self._receive(msg, now)
                    with self.lock:
                        if self.target and now - self.probe_at >= 10:
                            # 请求读取参数，不改参数、不解锁、不切换模式。
                            connection.mav.param_request_read_send(
                                *self.target, b"MAV_SYS_ID", -1)
                            self.probe_at = now
                        if self.target and now - self.state["heartbeat_at"] > settings.HEARTBEAT_MAX_AGE:
                            self.state.update(message="飞控心跳超时", bidirectional=False)
            except Exception as error:
                with self.lock:
                    self.state.update(message=str(error), heartbeat_at=0,
                                      bidirectional=False)
            finally:
                with self.lock:
                    connection, self.connection = self.connection, None
                    self.target = None
                if connection is not None:
                    connection.close()
            self.stop_event.wait(2)

    def _receive(self, msg, now):
        kind = msg.get_type()
        with self.changed:
            self.counts[kind] += 1
            source = (msg.get_srcSystem(), msg.get_srcComponent())
            if kind == "HEARTBEAT" and msg.autopilot == 12:
                if self.target is not None and source != self.target:
                    return
                self.target = source
                # 当前固件 Offboard 心跳 base_mode=17，旧 pymavlink 会返回 UNKNOWN。
                # custom_mode 的主模式字节为 6 才是 PX4 Offboard。
                main_mode = (msg.custom_mode >> 16) & 0xff
                mode = ("OFFBOARD" if (msg.base_mode & 1) and main_mode == 6
                        else mavutil.mode_string_v10(msg))
                self.state.update(heartbeat_at=now,
                                  mode=mode,
                                  base_mode=msg.base_mode,
                                  custom_mode=msg.custom_mode,
                                  armed=bool(msg.base_mode & 128),
                                  message="双向验证通过" if self.state["bidirectional"]
                                  else "心跳正常；上行尚未验证")
            elif source != self.target:
                return
            elif kind == "PARAM_VALUE":
                name = msg.param_id
                if isinstance(name, bytes):
                    name = name.decode("ascii", errors="replace")
                if name.rstrip("\0") == "MAV_SYS_ID" and now - self.probe_at < 5:
                    self.parameter_at = now
                    self.state.update(bidirectional=True, parameter_reply=msg.param_value,
                                      message="收到参数回复，双向验证通过")
            elif kind == "COMMAND_ACK":
                recipient = getattr(msg, "target_system", 0)
                if recipient not in (0, settings.SOURCE_SYSTEM):
                    return
                self.last_ack[msg.command] = (now, msg.result)
                self.changed.notify_all()
            elif kind == "STATUSTEXT":
                self.state["last_text"] = str(msg.text)
            elif kind == "ATTITUDE":
                self.state["attitude_deg"] = [round(math.degrees(getattr(msg, axis)), 2)
                                               for axis in ("roll", "pitch", "yaw")]
            elif kind == "LOCAL_POSITION_NED":
                self.state["local_position"] = {key: getattr(msg, key)
                    for key in ("x", "y", "z", "vx", "vy", "vz")}
                self.state["position_at"] = now
            elif kind == "ESTIMATOR_STATUS":
                self.state["estimator_flags"] = msg.flags
                self.state["estimator_at"] = now
            elif kind == "SYS_STATUS":
                self.state["battery_v"] = None if msg.voltage_battery == 65535 else msg.voltage_battery / 1000
            elif kind == "AUTOPILOT_VERSION":
                self.state["firmware_version_raw"] = msg.flight_sw_version
            elif kind == "EXTENDED_SYS_STATE":
                self.state["landed_state"] = msg.landed_state

    def command(self, number, params, timeout=3):
        """同一时间只允许一个命令等待回执；超时与拒绝分别报告。"""
        with self.command_lock:
            with self.changed:
                if not self.connection or not self.target:
                    raise RuntimeError("尚未连接 PX4")
                self.last_ack.pop(number, None)
                sent = time.monotonic()
                self.connection.mav.command_long_send(*self.target, number, 0, *params)
                deadline = sent + timeout
                while not self.stop_event.is_set():
                    ack = self.last_ack.get(number)
                    if ack and ack[0] >= sent:
                        return ack[1]
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self.changed.wait(remaining)
        raise TimeoutError("COMMAND_ACK 超时：上行/回程未验证，不能视为执行成功")

    def send_velocity(self, vx, vy, vz):
        """低层封装，主程序不会调用；飞行状态机必须另行验证。"""
        with self.lock:
            if not self.connection or not self.target:
                raise RuntimeError("飞控未连接")
            self.connection.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xffffffff,
                *self.target, 8, 3527, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

