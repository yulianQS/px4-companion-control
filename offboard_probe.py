"""未解锁 Offboard 模式验证：零速度、短时切换、恢复原 LOITER 模式。"""
import argparse
import os
import sys
import threading
import time

sys.dont_write_bytecode = True
import settings

if sys.prefix != settings.PYTHON_ENV:
    python = settings.PYTHON_ENV + '/bin/python'
    os.execv(python, [python, os.path.abspath(__file__)] + sys.argv[1:])

from link import PX4Link


class ProbeLink(PX4Link):
    def _receive(self, msg, now):
        super()._receive(msg, now)
        if msg.get_type() == 'HEARTBEAT' and msg.autopilot == 12:
            state = self.status()
            print('状态：', state['mode'], 'armed=', state['armed'],
                  'base_mode=', msg.base_mode, 'custom_mode=', msg.custom_mode,
                  flush=True)
        elif msg.get_type() == 'STATUSTEXT':
            print('飞控提示：', msg.text, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--switch-unarmed', action='store_true',
                        help='明确执行未解锁模式切换；不发送解锁命令')
    args = parser.parse_args()
    if not args.switch_unarmed:
        print('未执行硬件操作。经确认后使用 --switch-unarmed。')
        return 0

    link = ProbeLink()
    stop = threading.Event()
    errors = []
    worker = None
    attempted = False
    result = 2

    def check(require_sender=True):
        state = link.status()
        if not state['connected'] or state['armed'] is not False:
            raise RuntimeError('未明确确认心跳正常且未解锁，停止验证')
        if require_sender and errors:
            raise RuntimeError(errors[0])
        return state

    def wait_for_mode(name, timeout=5, require_sender=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = check(require_sender)
            if state['mode'] == name:
                print('心跳模式：', state['mode'],
                      'base_mode=', state.get('base_mode'),
                      'custom_mode=', state.get('custom_mode'))
                return
            time.sleep(0.1)
        raise RuntimeError('心跳未确认模式：%s，实际=%s，custom_mode=%s' %
                           (name, state['mode'], state.get('custom_mode')))

    def send_zero():
        try:
            deadline = time.monotonic() + 25
            while not stop.is_set() and time.monotonic() < deadline:
                check()
                link.send_velocity(0.0, 0.0, 0.0)
                stop.wait(0.05)
            if not stop.is_set():
                errors.append('零速度发送达到 25 秒上限')
        except Exception as error:
            errors.append(str(error))

    try:
        link.start()
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if link.status()['bidirectional']:
                break
            time.sleep(0.1)
        state = check()
        if not state['bidirectional'] or state['mode'] != 'LOITER':
            raise RuntimeError('要求双向验证成功，且初始模式为 LOITER')

        worker = threading.Thread(target=send_zero, daemon=True)
        worker.start()
        time.sleep(2)
        check()
        attempted = True
        ack = link.command(176, [1, 6, 0, 0, 0, 0, 0])
        print('Offboard 切换回执：', ack)
        if ack != 0:
            raise RuntimeError('飞控拒绝切换 Offboard')
        wait_for_mode('OFFBOARD')
        print('心跳确认 OFFBOARD，保持未解锁零速度 3 秒')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if check()['mode'] != 'OFFBOARD':
                raise RuntimeError('验证期间退出 Offboard')
            time.sleep(0.1)
        result = 0
    except (RuntimeError, TimeoutError, KeyboardInterrupt) as error:
        print('验证中止：', error)
    finally:
        if attempted:
            try:
                restored = False
                for attempt in range(3):
                    state = check(require_sender=False)
                    if state['mode'] == 'LOITER':
                        restored = True
                        break
                    try:
                        ack = link.command(176, [1, 4, 3, 0, 0, 0, 0])
                        print('恢复请求 %d 回执：%s' % (attempt + 1, ack))
                    except TimeoutError as error:
                        print('恢复请求 %d：%s；继续核对心跳' % (attempt + 1, error))
                    try:
                        wait_for_mode('LOITER', timeout=3, require_sender=False)
                        restored = True
                        break
                    except RuntimeError:
                        check(require_sender=False)
                if not restored:
                    raise RuntimeError('三次恢复请求后仍未确认 LOITER')
                print('心跳确认恢复 LOITER，仍未解锁')
            except Exception as error:
                result = 2
                print('恢复未确认，请在 QGC 查看当前模式：', error)
        stop.set()
        if worker:
            worker.join(timeout=2)
        link.stop()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
