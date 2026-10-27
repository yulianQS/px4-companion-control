"""Existing diagnostic CLI, now routed through C++; never opens UART in Python."""
import argparse
import json
import time
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.cpp_bridge import ServiceClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('doctor', 'motor-test'))
    parser.add_argument('--props-removed', action='store_true')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()
    if args.operation == 'motor-test' and not args.props_removed:
        parser.error('motor-test requires --props-removed')
    client = ServiceClient(args.host, args.port)
    client.start()
    owner = 'operator-terminal-client'
    try:
        deadline = time.monotonic() + 8
        while not client.status()['data']['service']['connected'] and time.monotonic() < deadline:
            time.sleep(.1)
        if args.operation == 'doctor':
            result = client.request('DOCTOR', {}, owner)
        else:
            claim = client.request('CLAIM', {}, owner)
            if not claim.get('ok'):
                raise RuntimeError(claim.get('message'))
            print('C++执行四路0.05输出，10秒窗口；请求仅在显式无桨确认后提交。', flush=True)
            result = client.request('MOTOR_TEST', {'props_removed': True}, owner, timeout=15)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('ok') else 2
    except (OSError, RuntimeError) as error:
        print('未确认完成：', error)
        return 2
    except KeyboardInterrupt:
        try:
            print(client.request('STOP_MOTOR_TEST', {}, owner, timeout=2))
        except RuntimeError as error:
            print('停止请求结果未知：', error)
        print('终端已中断并尝试停止；每路电机测试有效期1秒。请观察停转。')
        return 130
    finally:
        client.stop()


if __name__ == '__main__':
    raise SystemExit(main())
