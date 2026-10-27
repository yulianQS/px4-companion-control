#!/usr/bin/env python3
"""Start missing project processes once, detached from SSH, with persistent logs.

No control command, automatic arming, system service installation or parameter write.
ROS launch owns node respawn; this helper never kills an existing process.
"""
import datetime
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.request
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from web.server.config import RUNTIME
from log_storage import boot_id, recover_sessions, session_directory, utc, write_json
LOGS = ROOT / 'logs' / 'runtime'
SESSION = None


def processes(marker):
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = [part.decode(errors='replace') for part in (entry / 'cmdline').read_bytes().split(b'\0') if part]
            matches = any(part == marker or Path(part).name == marker for part in args)
            if marker == 'lib/drone_vision/': matches = any('/lib/drone_vision/' in part for part in args)
            if marker == 'ros2 launch drone_vision': matches = 'ros2' in [Path(part).name for part in args] and 'launch' in args and 'drone_vision' in args
            if matches and int(entry.name) != os.getpid():
                found.append(int(entry.name))
        except (OSError, ProcessLookupError):
            pass
    return found


def listening(port):
    with socket.socket() as probe:
        probe.settimeout(.3)
        return probe.connect_ex(('127.0.0.1', port)) == 0


def start(name, command):
    path = SESSION / (SESSION.name + '-' + name + '.log')
    native = ['--sync-tree', str(SESSION/'ros')] if name == 'vision' else []
    child = subprocess.Popen([sys.executable, '-B', str(ROOT/'scripts/runtime_logging.py'),
                              '--log', str(path), *native, '--', *command], cwd=ROOT,
                             env=dict(os.environ, DRONE_LOG_SESSION=SESSION.name,
                                      ROS_LOG_DIR=str(SESSION/'ros')),
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(.5)
    if child.poll() is not None:
        raise RuntimeError('%s 启动退出，日志：%s' % (name, path))
    manifest = json.loads((SESSION/'session.json').read_text())
    manifest.setdefault('started', {})[name] = dict(logger_pid=child.pid, log_file=path.name)
    write_json(SESSION/'session.json', manifest)
    print('%s 日志进程PID=%s，日志：%s' % (name, child.pid, path), flush=True)


def main():
    global SESSION
    LOGS.mkdir(parents=True, exist_ok=True)
    with (LOGS / 'startup.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        recover_sessions()
        SESSION = session_directory('runtime')
        write_json(SESSION/'session.json', dict(started_utc=utc(), boot_id=boot_id(),
                   runtime_session=SESSION.name, state='startup',
                   flight_records=str(ROOT/'flight_records'), mission_runs=str(ROOT/'mission_runs'),
                   existing_control=listening(RUNTIME['control']['port']),
                   existing_web=listening(RUNTIME['web']['port'])))
        # Never open a second flight UART, including when a foreign consumer owns it.
        if listening(RUNTIME['control']['port']):
            print('C++端口已监听，保留现有服务。')
        elif processes('drone_service'):
            raise RuntimeError('已有C++服务进程但8090未监听，请检查原服务，未启动重复进程。')
        else:
            users = subprocess.run(['fuser', RUNTIME['control']['device']], capture_output=True, text=True)
            if users.returncode != 1:
                raise RuntimeError('无法确认飞控串口空闲：' + users.stdout + users.stderr)
            start('control', [str(ROOT / 'build/drone_service'), '--logs', str(ROOT / 'flight_records')])
        if listening(RUNTIME['web']['port']):
            print('网页端口已监听，保留现有服务。')
        elif processes('web_control.py'):
            raise RuntimeError('已有网页进程但8081未监听，请检查原服务，未启动重复进程。')
        else:
            python = Path(RUNTIME['python_env']) / 'bin/python'
            start('web', [str(python if python.exists() else '/usr/bin/python3'), '-B', '-u', str(ROOT / 'web_control.py')])
        nodes = processes('lib/drone_vision/')
        launch = processes('ros2 launch drone_vision')
        if nodes or launch:
            print('视觉图已运行，保留现有节点：%s。' % (nodes or launch))
        else:
            start('vision', ['bash', str(ROOT / 'scripts/start_vision.sh'),
                             'config:=' + str(ROOT / 'config/vision.yaml')])
        deadline = time.monotonic() + 15
        last = ''
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen('http://127.0.0.1:%d/api/vision/status' % RUNTIME['web']['port'], timeout=1) as response:
                    state = json.load(response)
                if state.get('source') == 'ros2_cpp' and state.get('camera_ready'):
                    manifest = json.loads((SESSION/'session.json').read_text())
                    manifest.update(state='ready', ready_utc=utc())
                    write_json(SESSION/'session.json', manifest)
                    print('视觉有效帧已恢复。网页 http://192.168.137.3:8081/；启动不会自动启用实机控制。')
                    return
                last = str(state.get('message', state))
            except (OSError, ValueError) as error:
                last = str(error)
            time.sleep(.3)
        raise RuntimeError('视觉尚未恢复：%s；检查 %s，未停止其他进程。' % (last, LOGS))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        if SESSION is not None:
            manifest = json.loads((SESSION/'session.json').read_text())
            manifest.update(state='startup_failed', error=str(error), ended_utc=utc())
            write_json(SESSION/'session.json', manifest)
        raise
