"""未解锁Offboard进程退出/重新连接验证；没有解锁或非零速度指令。"""
import argparse
import os
from pathlib import Path
import selectors
import subprocess
import sys
import threading
import time

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import settings
if sys.prefix != settings.PYTHON_ENV:
    executable = settings.PYTHON_ENV + '/bin/python'
    os.execv(executable, [executable, os.path.abspath(__file__)] + sys.argv[1:])
from link import PX4Link


# ==================== 连接与状态确认 ====================
class CheckLink(PX4Link):
    def __init__(self):
        super().__init__()
        self.shell_lock = threading.Lock()
        self.shell_parts = []

    def _receive(self, msg, now):
        super()._receive(msg, now)
        if msg.get_type() == 'SERIAL_CONTROL':
            with self.shell_lock:
                self.shell_parts.append(bytes(msg.data[:msg.count]).decode('utf-8', errors='replace'))

    def checked(self):
        status = self.status()
        if not status['connected'] or status['armed'] is not False:
            raise RuntimeError('要求心跳正常且明确未解锁')
        return status

    def connect_checked(self):
        self.start()
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            status = self.status()
            if status['connected'] and status['armed'] is not False:
                raise RuntimeError('飞控不是未解锁状态')
            if status['bidirectional']:
                return self.checked()
            time.sleep(0.05)
        raise RuntimeError('双向连接超时')

    def wait_mode(self, name, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.checked()['mode'] == name:
                return True
            time.sleep(0.05)
        return False

    def signal_lost(self):
        with self.shell_lock:
            self.shell_parts.clear()
        command = b'listener failsafe_flags -n 1\n'
        with self.lock:
            self.connection.mav.serial_control_send(10, 22, 0, 0, len(command),
                                                   list(command)+[0]*(70-len(command)))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.checked()
            time.sleep(0.05)
        with self.shell_lock:
            text = ''.join(self.shell_parts)
        for line in text.splitlines():
            if line.strip().startswith('offboard_control_signal_lost:'):
                value = line.split(':', 1)[1].strip()
                if value in ('True', 'False'):
                    return value == 'True'
        raise RuntimeError('未收到完整失联标志，不假设结果')

    def release(self):
        try:
            with self.lock:
                if self.connection:
                    self.connection.mav.serial_control_send(10, 0, 0, 0, 0, [0]*70)
        finally:
            self.stop()


# ==================== 有时限的零速度流 ====================
def zero_stream(link, stop, errors, duration=45):
    try:
        deadline = time.monotonic() + duration
        while not stop.is_set() and time.monotonic() < deadline:
            link.checked()
            link.send_velocity(0.0, 0.0, 0.0)
            stop.wait(0.05)
        if not stop.is_set():
            errors.append('零速度流达到时间上限')
    except Exception as error:
        errors.append(str(error))


def child_stream():
    link = CheckLink()
    try:
        status = link.connect_checked()
        if status['mode'] != 'OFFBOARD':
            raise RuntimeError('子进程要求已处于未解锁Offboard')
        link.send_velocity(0.0, 0.0, 0.0)
        print('STREAM_READY', flush=True)
        errors = []
        zero_stream(link, threading.Event(), errors, duration=20)
        if errors:
            raise RuntimeError(str(errors))
    finally:
        link.release()


# ==================== 进程退出与恢复测试 ====================
def restart_check():
    link = CheckLink()
    child = None
    stop = threading.Event()
    errors = []
    worker = None
    mode_attempted = False
    try:
        initial = link.connect_checked()
        if initial['mode'] != 'LOITER':
            raise RuntimeError('要求初始未解锁LOITER')
        worker = threading.Thread(target=zero_stream, args=(link, stop, errors), daemon=True)
        worker.start()
        time.sleep(2)
        link.checked()
        if errors:
            raise RuntimeError(str(errors))
        mode_attempted = True
        ack = link.command(176, [1, 6, 0, 0, 0, 0, 0])
        if ack != 0 or not link.wait_mode('OFFBOARD'):
            raise RuntimeError('未确认进入Offboard')
        stop.set()
        worker.join(timeout=2)
        link.release()
        link = None

        # 仅创建并终止本次子进程；不寻找或杀死其他工程/系统进程。
        child = subprocess.Popen([sys.executable, __file__, '--stream-child'],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1)
        with selectors.DefaultSelector() as selector:
            selector.register(child.stdout, selectors.EVENT_READ)
            if not selector.select(timeout=14):
                raise RuntimeError('子进程就绪超时')
            ready = child.stdout.readline().strip()
        if ready != 'STREAM_READY':
            raise RuntimeError('子进程未就绪: ' + ready)
        print('PASS: 新进程重新打开UART7并发送零速度', flush=True)
        time.sleep(2)
        child.kill()
        child.wait(timeout=3)
        child.stdout.close()
        child = None
        print('本次子进程已终止；检查UART释放与指令中断', flush=True)

        link = CheckLink()
        status = link.connect_checked()
        time.sleep(3)
        if link.signal_lost() is not True:
            raise RuntimeError('进程退出后没有确认指令中断')
        print('PASS: UART7重新打开，失联标志=True', flush=True)
        stop = threading.Event()
        errors = []
        worker = threading.Thread(target=zero_stream, args=(link, stop, errors), daemon=True)
        worker.start()
        time.sleep(2)
        if errors or link.signal_lost() is not False:
            raise RuntimeError('重新发送后没有确认信号恢复')
        print('PASS: 零速度恢复，失联标志=False', flush=True)
        print('接收计数:', link.checked()['counts'], flush=True)
    finally:
        stop.set()
        if worker:
            worker.join(timeout=2)
        if child is not None:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=3)
            child.stdout.close()
        if mode_attempted and link is None:
            link = CheckLink()
            link.connect_checked()
        if link is not None:
            try:
                if mode_attempted:
                    restored = False
                    for retry in range(3):
                        if link.wait_mode('LOITER', timeout=0.1):
                            restored = True
                            break
                        try:
                            print('恢复回执:', link.command(176, [1,4,3,0,0,0,0], timeout=1), flush=True)
                        except TimeoutError:
                            print('恢复回执超时，继续核对心跳', flush=True)
                        if link.wait_mode('LOITER'):
                            restored = True
                            break
                    if not restored:
                        raise RuntimeError('未确认恢复LOITER，请现场检查模式')
                    print('FINAL: 心跳确认未解锁LOITER', flush=True)
            finally:
                link.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unarmed-restart-check', action='store_true',
                        help='明确执行未解锁模式切换与本次子进程退出验证')
    parser.add_argument('--stream-child', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.stream_child:
        child_stream()
    elif args.unarmed_restart_check:
        restart_check()
    else:
        print('未操作硬件；执行测试需显式添加 --unarmed-restart-check。')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, TimeoutError, KeyboardInterrupt) as error:
        print('测试未通过:', error, file=sys.stderr)
        raise SystemExit(2)
