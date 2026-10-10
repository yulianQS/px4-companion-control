#!/usr/bin/env python3
"""整合入口：python3 test.py。默认只有监测和控制预览。"""
import os
import signal
import sys
import threading
import time
sys.dont_write_bytecode = True
import settings

if sys.prefix != settings.PYTHON_ENV:
    interpreter = os.path.join(settings.PYTHON_ENV, "bin", "python")
    os.execv(interpreter, [interpreter, os.path.abspath(__file__)])


def main():
    # 保留 cv_work 的驱动/识别实现，通过回调接入本工程；不复制两份视觉源码。
    sys.path.insert(0, settings.VISION_DIRECTORY)
    import config as vision_config
    vision_config.PROCESS_FPS = min(vision_config.PROCESS_FPS, settings.VISION_FPS)
    # 厂家 C 库输出丢弃，Python 错误和网址仍保留；不生成运行日志。
    sys.stdout.flush()
    sys.stderr.flush()
    sys.stdout = os.fdopen(os.dup(1), "w", buffering=1, encoding="utf-8")
    sys.stderr = os.fdopen(os.dup(2), "w", buffering=1, encoding="utf-8")
    with open(os.devnull, "w") as sink:
        os.dup2(sink.fileno(), 1)
        os.dup2(sink.fileno(), 2)
    import cv2
    from vision import FrameStore, VisionService
    from link import PX4Link
    from control import preview
    from dashboard import create_server
    import subprocess
    import ipaddress

    cv2.setNumThreads(1)
    px4 = PX4Link()
    store = FrameStore()
    detection_lock = threading.Lock()
    detection = {}

    def on_detection(result):
        with detection_lock:
            detection.clear()
            detection.update(result, received_at=time.monotonic())

    vision = VisionService(store, on_detection)

    def snapshot():
        now = time.monotonic()
        with detection_lock:
            result = dict(detection)
        status = px4.status()
        proposal = preview(result, status, now)
        if result:
            age = now - result.pop("received_at")
            result["age_s"] = round(age, 2)
            if age > settings.VISION_MAX_AGE:
                result.update(target_found=False, center=None, message="视觉数据过期")
        return {"vision": result, "px4": status, "control": proposal}

    server = create_server(store, vision.stop_event, snapshot)

    def stop_requested(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_requested)
    print("默认只读 + 控制预览；不会解锁或切换模式。")
    try:
        addresses = subprocess.check_output(["hostname", "-I"], text=True, timeout=2).split()
        print("浏览器访问地址：")
        for address in addresses:
            if ipaddress.ip_address(address).version == 4:
                print("  http://%s:%d/" % (address, settings.WEB_PORT))
    except (OSError, ValueError, subprocess.SubprocessError):
        print("无法读取局域网地址，请检查网络。")
    try:
        px4.start()
        vision.start()
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        vision.stop()
        px4.stop()
        server.server_close()
        print("摄像头、串口、网页已释放。")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("启动失败：", error, file=sys.stderr)
        raise SystemExit(1)
