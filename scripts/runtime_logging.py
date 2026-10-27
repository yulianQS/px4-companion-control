#!/usr/bin/env python3
"""Capture one process's stdout/stderr; fsync on this worker, never its UI thread."""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time
from log_storage import boot_id, checkpoint, checkpoint_tree, sync_directory, utc, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--sync-tree', type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('missing child command')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    metadata = dict(started_utc=utc(), monotonic_start_s=time.monotonic(), boot_id=boot_id(),
                    logger_pid=os.getpid(),
                    runtime_session=os.environ.get('DRONE_LOG_SESSION', ''),
                    command=command, state='running', log_file=args.log.name)
    child = None
    result = 1
    try:
        with args.log.open('xb') as output:
            output.write((json.dumps(metadata, ensure_ascii=False)+'\n').encode())
            checkpoint(output)
            sync_directory(args.log.parent)
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     env=dict(os.environ, DRONE_LOG_CAPTURED='1'))
            metadata['pid'] = child.pid
            try:
                write_json(args.log.with_suffix('.json'), metadata)
            except OSError as error:
                metadata['error'] = str(error)
            def forward(signum, _frame):
                if child.poll() is None:
                    os.killpg(child.pid, signum)
            signal.signal(signal.SIGTERM, forward)
            signal.signal(signal.SIGINT, forward)
            synced = time.monotonic()
            recording = True
            native_versions = {}
            with selectors.DefaultSelector() as selector:
                selector.register(child.stdout, selectors.EVENT_READ)
                while selector.get_map():
                    for key, _ in selector.select(timeout=.2):
                        data = os.read(key.fileobj.fileno(), 65536)
                        if data:
                            if recording:
                                try:
                                    output.write(data)
                                except OSError as error:
                                    metadata['error'] = str(error)
                                    recording = False
                        else:
                            selector.unregister(key.fileobj)
                    if time.monotonic()-synced >= 1:
                        try:
                            if recording: checkpoint(output)
                        except OSError as error:
                            metadata['error'] = str(error)
                            recording = False  # Keep draining; logging must not terminate control.
                        if args.sync_tree:
                            try:
                                checkpoint_tree(args.sync_tree, native_versions)
                            except OSError as error:
                                metadata['native_log_error'] = str(error)
                        synced = time.monotonic()
            result = child.wait()
            if recording:
                output.write(('\n'+json.dumps(dict(event='process_exited', utc_iso=utc(), returncode=result))+'\n').encode())
                checkpoint(output)
            if args.sync_tree: checkpoint_tree(args.sync_tree, native_versions)
            child.stdout.close()
    except BaseException as error:
        metadata['error'] = str(error)
        raise
    finally:
        metadata.update(ended_utc=utc(), monotonic_end_s=time.monotonic(), returncode=result,
                        state='log_error' if 'error' in metadata or 'native_log_error' in metadata
                        else 'completed' if result == 0 else 'interrupted')
        write_json(args.log.with_suffix('.json'), metadata)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
