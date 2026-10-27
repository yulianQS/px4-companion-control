"""Versioned mission graph validation and atomic persistence. IDs never mean indices."""
import copy
import json
import math
import os
import re
import tempfile
import threading
import uuid
from pathlib import Path
from .mission_catalog import BY_TYPE
from .config import ROOT

ID = re.compile(r'^[A-Za-z0-9_-]{1,100}$')


def valid_id(value):
    return isinstance(value, str) and ID.fullmatch(value) is not None


def validate(document):
    if not isinstance(document, dict) or document.get('version') != 1:
        raise ValueError('任务需要 version=1')
    blocks = document.get('blocks')
    if not isinstance(blocks, list) or len(blocks) > 200:
        raise ValueError('任务最多包含 200 个积木')
    result = dict(version=1, id=document.get('id') or uuid.uuid4().hex,
                  name=document.get('name', '未命名比赛任务'), entry=document.get('entry'), blocks=[])
    if not valid_id(result['id']) or not isinstance(result['name'], str) or not 1 <= len(result['name'].strip()) <= 100:
        raise ValueError('任务 ID 或名称无效')
    ids = set()
    for raw in blocks:
        if not isinstance(raw, dict) or not valid_id(raw.get('id')) or raw['id'] in ids:
            raise ValueError('积木 ID 无效或重复')
        ids.add(raw['id'])
        definition = BY_TYPE.get(raw.get('type'))
        if definition is None:
            raise ValueError('未知积木类型')
        supplied = raw.get('params', {})
        if not isinstance(supplied, dict) or set(supplied) - {f['key'] for f in definition['fields']}:
            raise ValueError('积木参数存在未知字段')
        params = {}
        for f in definition['fields']:
            value = copy.deepcopy(supplied.get(f['key'], f['default']))
            kind = f['kind']
            if kind == 'optional_number' and value is None:
                params[f['key']] = None
                continue
            if kind in ('number', 'integer', 'optional_number'):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f['label'] + ' 必须为有限数值')
                if kind == 'integer' and int(value) != value:
                    raise ValueError(f['label'] + ' 必须为整数')
                if not f.get('min', -math.inf) <= value <= f.get('max', math.inf):
                    raise ValueError(f['label'] + ' 超出支持范围')
            elif kind in ('text', 'ref', 'select'):
                if not isinstance(value, str) or len(value) > 2000:
                    raise ValueError(f['label'] + ' 格式无效')
                if kind == 'select' and value not in f['options']:
                    raise ValueError(f['label'] + ' 选项无效')
            params[f['key']] = value
        if 'waypoints' in params:
            waypoints = params['waypoints']
            if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 100:
                raise ValueError('航点数量需要 1～100')
            if any(not isinstance(row, list) or len(row) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 10000 for v in row) for row in waypoints):
                raise ValueError('航点需要有限的 XYZ 厘米坐标')
        timeout = raw.get('timeout', 120)
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 2 <= timeout <= 3600:
            raise ValueError('积木超时需要 2～3600 秒')
        if raw.get('branches', {}) != {}:
            raise ValueError('当前积木不支持条件分支')
        name = raw.get('name', definition['name'])
        if not isinstance(name, str) or not 1 <= len(name) <= 100:
            raise ValueError('积木名称无效')
        result['blocks'].append(dict(id=raw['id'], type=raw['type'], name=name, params=params,
                                     status='pending', next=raw.get('next'), branches={}, timeout=timeout,
                                     auto_next=raw.get('auto_next', True) is True))
    if result['entry'] is None and blocks:
        result['entry'] = blocks[0]['id']
    refs = [result['entry']]
    for block in result['blocks']:
        refs.extend([block['next'], block['params'].get('lost_target')])
    if any(ref not in ('', None) and ref not in ids for ref in refs):
        raise ValueError('任务引用不存在')
    if len(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()) > 262144:
        raise ValueError('任务 JSON 超过 256KB')
    return result


def execution_issues(document):
    issues = []
    if not document['blocks']:
        return ['请先添加任务积木']
    for b in document['blocks']:
        d = BY_TYPE[b['type']]
        if not d['executable']:
            issues.append(b['name'] + '：' + d['reason'])
        if 'target_id' in b['params'] and b['type'] != 'jump' and b['params']['target_id'] != 0:
            issues.append(b['name'] + '：现有视觉接口仅支持 AprilTag ID 0')
    return issues


class MissionStore:
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory else ROOT / 'missions'
        self.lock = threading.Lock()

    def path(self, id_):
        if not valid_id(id_):
            raise ValueError('任务 ID 无效')
        return self.directory / (id_ + '.json')

    def list(self):
        if not self.directory.exists():
            return []
        rows = []
        with self.lock:
            for path in self.directory.glob('*.json'):
                try:
                    value = self._load(path)
                    rows.append(dict(id=value['id'], name=value['name'], blocks=len(value['blocks']), modified=path.stat().st_mtime))
                except (OSError, ValueError, KeyError):
                    continue
        return sorted(rows, key=lambda r: r['modified'], reverse=True)

    @staticmethod
    def _load(path):
        with path.open('rb') as stream:
            data = stream.read(262145)
        if len(data) > 262144:
            raise ValueError('任务文件过大')
        return validate(json.loads(data))

    def load(self, id_):
        with self.lock:
            return self._load(self.path(id_))

    def save(self, value):
        value = validate(value)
        with self.lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            pending = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.directory, prefix='pending-', suffix='.tmp', delete=False) as stream:
                    pending = stream.name
                    json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(pending, self.path(value['id']))
            finally:
                if pending and os.path.exists(pending):
                    os.unlink(pending)
        return value
