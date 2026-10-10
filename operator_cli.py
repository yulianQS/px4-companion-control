"""独立诊断/无桨电机测试；先退出 test.py，不能同时占用 UART7。"""
import argparse
import json
import os
import sys
import time
sys.dont_write_bytecode = True
import settings
if sys.prefix != settings.PYTHON_ENV:
    os.execv(settings.PYTHON_ENV + "/bin/python",
             [settings.PYTHON_ENV + "/bin/python", os.path.abspath(__file__)] + sys.argv[1:])
from link import PX4Link


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("doctor", "motor-test"))
    parser.add_argument("--props-removed", action="store_true", help="明确无桨台架测试")
    args = parser.parse_args()
    if args.operation == "motor-test" and not args.props_removed:
        parser.error("无桨测试需显式加入 --props-removed")
    link = PX4Link()
    started = False
    try:
        link.start()
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if link.status().get("bidirectional"):
                break
            time.sleep(0.1)
        print(json.dumps(link.status(), ensure_ascii=False, indent=2))
        if args.operation == "doctor":
            try:
                result = link.command(512, [148, 0, 0, 0, 0, 0, 0])
                print("读取固件信息 COMMAND_ACK result=", result)
            except (TimeoutError, RuntimeError) as error:
                print(error)
                return 2
            return 0 if link.status()["bidirectional"] and result == 0 else 2
        status = link.status()
        if not status["connected"] or not status["bidirectional"]:
            raise RuntimeError("双向参数读取未通过，拒绝启动电机测试")
        if status["armed"] is not False:
            raise RuntimeError("飞控不是明确未解锁状态")
        print("四路相同低输出 0.05，测试窗口 10 秒；Ctrl+C 中断")
        deadline = time.monotonic() + 10
        started = True
        while time.monotonic() < deadline:
            status = link.status()
            if not status["connected"] or status["armed"] is not False:
                raise RuntimeError("心跳失联或解锁状态改变")
            for motor in (1, 2, 3, 4):
                if time.monotonic() >= deadline:
                    break
                result = link.command(310, [0.05, 1, 0, 0, motor, 0, 0], timeout=0.3)
                if result != 0:
                    raise RuntimeError("电机测试被拒绝 result=%s" % result)
            time.sleep(0.1)
        print("测试窗口结束")
        return 0
    except (RuntimeError, TimeoutError) as error:
        print("终止：", error)
        return 2
    except KeyboardInterrupt:
        print("手动中断")
        return 130
    finally:
        if started:
            for motor in (1, 2, 3, 4):
                try:
                    result = link.command(310, [float("nan"), 0, 0, 0, motor, 0, 0], timeout=0.3)
                    print("停止回执 Motor %d: %d" % (motor, result))
                except Exception as error:
                    print("停止回执未确认：", error)
            print("必须观察全部停转；通信丢失时测试命令有效期为 1 秒")
        link.stop()


if __name__ == "__main__":
    raise SystemExit(main())
