"""独立未解锁状态机验证：只允许零速度，不包含解锁或飞行入口。"""
import argparse
import os
from pathlib import Path
import sys
import threading
import time

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import settings
if sys.prefix != settings.PYTHON_ENV:
    executable = settings.PYTHON_ENV + '/bin/python'
    os.execv(executable, [executable, os.path.abspath(__file__)] + sys.argv[1:])
from offboard_restart_check import CheckLink


# ==================== 目标时限与停止锁存 ====================
class ZeroTarget:
    """锁存停止：恢复连接或提交新目标均不能自动重新激活会话。"""
    def __init__(self, ttl=0.5):
        self.ttl = ttl
        self.expires = 0.0
        self.reason = None
        self.lock = threading.Lock()

    def refresh(self, now):
        with self.lock:
            if self.reason is not None:
                raise RuntimeError('会话已停止: ' + self.reason)
            self.expires = now + self.ttl

    def stop(self, reason):
        with self.lock:
            if self.reason is None:
                self.reason = reason

    def check(self, now, connected=True, armed=False):
        with self.lock:
            if self.reason is None:
                if not connected:
                    self.reason = '心跳中断'
                elif armed is not False:
                    self.reason = '飞控不再明确未解锁'
                elif now >= self.expires:
                    self.reason = '目标过期'
            return self.reason


# ==================== 无硬件边界验证 ====================
def self_check():
    for cause in ('目标过期', '人工停止', '心跳中断', '意外解锁'):
        target = ZeroTarget()
        target.refresh(10.0)
        assert target.check(10.1) is None
        if cause == '目标过期':
            assert target.check(10.5) == cause
        elif cause == '人工停止':
            target.stop(cause)
        elif cause == '心跳中断':
            assert target.check(10.2, connected=False) == cause
        else:
            assert target.check(10.2, armed=True) is not None
        assert target.check(10.3) is not None
        try:
            target.refresh(11.0)
        except RuntimeError:
            pass
        else:
            raise AssertionError('已停止目标被重新激活')
        print('PASS: 边界与停止锁存 /', cause, flush=True)


# ==================== 未解锁会话与确定性收尾 ====================
def session(link, manual_stop, wait_interrupt=False, inject_exception=False):
    target = ZeroTarget()
    target.refresh(time.monotonic())
    finish = threading.Event()
    errors = []
    sent = [0]
    attempted = False
    state = 'PRESTREAM'

    def sender():
        try:
            deadline = time.monotonic() + 20
            while not finish.is_set():
                status = link.status()
                reason = target.check(time.monotonic(), status['connected'], status['armed'])
                if time.monotonic() >= deadline:
                    target.stop('会话达到时间上限')
                    reason = target.reason
                if reason:
                    return
                # 固定为零；本脚本不接受非零目标，也不发送解锁命令。
                link.send_velocity(0.0, 0.0, 0.0)
                sent[0] += 1
                finish.wait(0.05)
        except Exception as error:
            errors.append(str(error))
            target.stop('发送失败')

    def checked():
        status = link.checked()
        if errors or target.reason:
            raise RuntimeError(str(errors) if errors else target.reason)
        if state == 'ACTIVE' and status['mode'] != 'OFFBOARD':
            target.stop('意外退出Offboard')
            raise RuntimeError(target.reason)
        return status

    def refresh_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            checked()
            target.refresh(time.monotonic())
            time.sleep(0.05)

    worker = threading.Thread(target=sender, daemon=True)
    worker.start()
    try:
        print('STATE:', state, flush=True)
        refresh_for(2)
        attempted = True
        # 切换等待期间也必须刷新目标；由单独的短时刷新线程维持。
        refresh_stop = threading.Event()
        def refresh_during_switch():
            try:
                while not refresh_stop.wait(0.05):
                    checked()
                    target.refresh(time.monotonic())
            except Exception as error:
                errors.append(str(error))
        refresher = threading.Thread(target=refresh_during_switch, daemon=True)
        refresher.start()
        try:
            ack = link.command(176, [1, 6, 0, 0, 0, 0, 0])
            if ack != 0 or not link.wait_mode('OFFBOARD'):
                raise RuntimeError('未确认Offboard接管')
        finally:
            refresh_stop.set()
            refresher.join(timeout=2)
        checked()
        state = 'ACTIVE'
        print('STATE:', state, '/ 心跳确认未解锁OFFBOARD', flush=True)
        if wait_interrupt:
            print('INTERRUPT_READY: 等待终端Ctrl+C；最多10秒', flush=True)
            refresh_for(10)
            raise RuntimeError('未收到终端Ctrl+C')
        refresh_for(1)
        if inject_exception:
            print('FAULT_INJECTED: 主线程受控RuntimeError', flush=True)
            raise RuntimeError('受控测试异常：验证finally收尾')
        expected = '人工停止' if manual_stop else '目标过期'
        if manual_stop:
            target.stop(expected)
        deadline = time.monotonic() + 1.5
        while target.reason is None and time.monotonic() < deadline:
            link.checked()
            time.sleep(0.02)
        worker.join(timeout=1)
        if target.reason != expected or worker.is_alive() or errors:
            raise RuntimeError('停止条件不符合预期: %s / %s' % (target.reason, errors))
        before = sent[0]
        try:
            target.refresh(time.monotonic())
        except RuntimeError:
            pass
        else:
            raise RuntimeError('停止后错误接受新目标')
        time.sleep(0.15)
        if sent[0] != before:
            raise RuntimeError('停止后仍发送目标')
        state = 'STOPPING'
        print('PASS:', expected, '/ 停止发送且拒绝目标重放; 发送条数=', before, flush=True)
    finally:
        target.stop('会话收尾')
        finish.set()
        worker.join(timeout=2)
        if attempted:
            restored = False
            for _ in range(3):
                if link.wait_mode('LOITER', timeout=0.1):
                    restored = True
                    break
                try:
                    print('恢复回执:', link.command(176, [1, 4, 3, 0, 0, 0, 0], timeout=1), flush=True)
                except TimeoutError:
                    print('恢复回执超时，核对心跳', flush=True)
                if link.wait_mode('LOITER'):
                    restored = True
                    break
            if not restored:
                raise RuntimeError('未确认恢复LOITER，请现场检查')
            state = 'CLOSED'
            print('FINAL: 未解锁LOITER', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--unarmed-session-check', action='store_true')
    parser.add_argument('--unarmed-interrupt-check', action='store_true',
                        help='未解锁零速度接管后等待终端Ctrl+C，始终执行恢复收尾')
    parser.add_argument('--unarmed-exception-check', action='store_true',
                        help='未解锁零速度接管后主动抛出RuntimeError，验证恢复收尾')
    args = parser.parse_args()
    if args.self_check:
        self_check()
    elif args.unarmed_session_check or args.unarmed_interrupt_check or args.unarmed_exception_check:
        link = CheckLink()
        try:
            status = link.connect_checked()
            if status['mode'] != 'LOITER':
                raise RuntimeError('要求初始未解锁LOITER')
            if args.unarmed_exception_check:
                session(link, False, inject_exception=True)
            elif args.unarmed_interrupt_check:
                session(link, False, wait_interrupt=True)
            else:
                for manual in (False, True):
                    session(link, manual)
            print('接收计数:', link.checked()['counts'], flush=True)
        finally:
            link.release()
    else:
        print('未操作硬件；--self-check离线检查，--unarmed-session-check执行未解锁验证。')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('终端中断：已执行会话收尾；恢复结果以上方心跳确认为准。', file=sys.stderr)
        raise SystemExit(130)
    except (RuntimeError, TimeoutError) as error:
        print('验证未通过:', error, file=sys.stderr)
        raise SystemExit(2)
