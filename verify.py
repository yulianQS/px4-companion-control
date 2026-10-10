"""纯软件验证：PTY 模拟飞控；不访问真实 UART7、不控制电机。"""
import os
import pty
import select
import sys
import threading
import time
import tty
sys.dont_write_bytecode = True
import settings
from link import PX4Link
from control import preview
from pymavlink import mavutil

assert mavutil.mavlink.WIRE_PROTOCOL_VERSION == "2.0", "工程应使用 MAVLink 2"


def require(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("条件等待超时")


master, slave = pty.openpty()
tty.setraw(slave)
os.set_blocking(master, False)
finished = threading.Event()
messages = []


class Writer:
    def write(self, data):
        return os.write(master, data)


def fake_px4():
    mav = mavutil.mavlink.MAVLink(Writer(), srcSystem=1, srcComponent=1)
    heartbeat_at = 0
    while not finished.is_set():
        if time.monotonic() - heartbeat_at > 0.2:
            mav.heartbeat_send(2, 12, 29, 50593792, 3)
            heartbeat_at = time.monotonic()
        if select.select([master], [], [], 0.02)[0]:
            data = os.read(master, 4096)
            for value in data:
                msg = mav.parse_char(bytes([value]))
                if msg is None:
                    continue
                messages.append(msg)
                if msg.get_type() == "PARAM_REQUEST_READ":
                    mav.param_value_send(b"MAV_SYS_ID", 1, 9, 1, 0)
                elif msg.get_type() == "COMMAND_LONG":
                    if msg.command == 512:
                        mav.command_ack_send(msg.command, 0)
                    elif msg.command == 310:
                        mav.command_ack_send(msg.command, 2)


emulator = threading.Thread(target=fake_px4, daemon=True)
link = PX4Link(os.ttyname(slave))
try:
    emulator.start()
    link.start()
    require(lambda: link.status()["bidirectional"])
    assert link.status()["target"] == (1, 1)
    assert link.status()["armed"] is False
    assert link.command(512, [148, 0, 0, 0, 0, 0, 0]) == 0
    assert link.command(310, [float("nan"), 0, 0, 0, 1, 0, 0]) == 2
    try:
        link.command(999, [0] * 7, timeout=0.15)
        raise AssertionError("应该超时")
    except TimeoutError:
        pass
    link.send_velocity(0.1, -0.1, 0)
    require(lambda: any(m.get_type() == "SET_POSITION_TARGET_LOCAL_NED" for m in messages))
    packet = next(m for m in messages if m.get_type() == "SET_POSITION_TARGET_LOCAL_NED")
    assert packet.coordinate_frame == 8 and packet.type_mask == 3527
    assert abs(packet.vx - 0.1) < 1e-6 and packet.vz == 0
    finished.set()
    emulator.join(1)
    require(lambda: not link.status()["connected"], timeout=4)
    assert not link.status()["bidirectional"]
    print("PASS: 模拟双向串口、接受/拒绝/超时、BODY 速度编码、心跳失联")
finally:
    finished.set()
    link.stop()
    emulator.join(1)
    os.close(master)
    os.close(slave)

now = time.monotonic()
detection = dict(camera_ready=True, target_found=True, center=[400, 300],
                 width=640, height=480, received_at=now, cpu_temp_c=60)
state = dict(connected=True, bidirectional=True, estimator_at=now, estimator_flags=12)
old_matrix = settings.IMAGE_TO_BODY
try:
    assert preview(detection, state, now)["body_velocity_m_s"] == [0, 0, 0]
    settings.IMAGE_TO_BODY = ((1, 0), (0, 1))
    assert preview(detection, state, now)["body_velocity_m_s"][0] < 0
    for changes in (dict(target_found=False), dict(received_at=now - 1), dict(cpu_temp_c=90)):
        assert preview(dict(detection, **changes), state, now)["body_velocity_m_s"] == [0, 0, 0]
    assert preview(detection, dict(state, bidirectional=False), now)["body_velocity_m_s"] == [0, 0, 0]
    assert preview(detection, dict(state, estimator_flags=164), now)["body_velocity_m_s"] == [0, 0, 0]
finally:
    settings.IMAGE_TO_BODY = old_matrix
print("PASS: 未标定、目标丢失、视觉过期、过热、上行失效、定位无效均抑制控制")

sys.path.insert(0, settings.VISION_DIRECTORY)
import cv2
import numpy as np
from vision import TagDetector
frame = np.full((480, 640, 3), 255, np.uint8)
marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 160)
frame[100:260, 100:260] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
result = TagDetector().detect(frame)
assert result["target_found"] and all(abs(v - 179.5) < 1 for v in result["center"])
print("PASS: 真实 AprilTag 检测器识别合成 36h11 ID0，中心误差 <1 像素")
