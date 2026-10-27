"""Project-local log identity and durable checkpoints; no global settings."""
import datetime
import json
import os
from pathlib import Path
import uuid

ROOT = Path(__file__).resolve().parents[1]


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return 'unknown'


def identity():
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    return f'{stamp}_b{boot_id()[:8]}_p{os.getpid()}_{uuid.uuid4().hex[:8]}'


def sync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def checkpoint(file):
    file.flush()
    os.fsync(file.fileno())


def checkpoint_tree(directory, previous):
    """Sync changed native ROS files as well as captured console output."""
    directory = Path(directory)
    if not directory.exists():
        return
    sync_directory(directory.parent)
    for path in [directory, *directory.rglob('*')]:
        if path.is_symlink():
            continue
        stat = path.stat()
        version = (stat.st_mtime_ns, stat.st_size)
        if previous.get(str(path)) == version:
            continue
        if path.is_dir():
            sync_directory(path)
        elif path.is_file():
            with path.open('rb') as file:
                os.fsync(file.fileno())
        previous[str(path)] = version


def write_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temporary.open('x', encoding='utf-8') as output:
        json.dump(data, output, ensure_ascii=False, indent=2)
        output.write('\n')
        checkpoint(output)
    os.replace(temporary, path)
    sync_directory(path.parent)


def session_directory(category, label=''):
    parent = ROOT / 'logs' / category
    parent.mkdir(parents=True, exist_ok=True)
    directory = parent / (identity() + ('-' + label if label else ''))
    directory.mkdir()
    sync_directory(parent.parent)
    sync_directory(parent)
    return directory


def recover_sessions():
    """Mark dead console writers without changing or deleting their raw output."""
    for path in (ROOT/'logs/runtime').glob('*/*.json'):
        try:
            metadata = json.loads(path.read_text(encoding='utf-8'))
            if metadata.get('state') != 'running' or 'command' not in metadata:
                continue
            alive = False
            if metadata.get('boot_id') == boot_id() and metadata.get('logger_pid', 0) > 0:
                try:
                    os.kill(metadata['logger_pid'], 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                metadata.update(state='interrupted', recovered_utc=utc(),
                                reason='logger no longer running; no clean exit recorded')
                write_json(path, metadata)
        except (OSError, ValueError):
            continue


if __name__ == '__main__':
    print(session_directory('ros'), flush=True)
