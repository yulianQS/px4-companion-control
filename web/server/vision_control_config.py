"""Persistent visual-controller defaults; no flight connection or control loop."""
import json
import math
import os
from pathlib import Path
import tempfile
from .config import ROOT

DEFAULTS = dict(yaw_gain=.3, yaw_ki=0., yaw_kd=0., yaw_speed_rad_s=0.,
                vertical_gain=.15, vertical_ki=0., vertical_kd=0., vertical_speed_m_s=0.,
                distance_gain=.3, distance_ki=0., distance_kd=0., forward_speed_m_s=0.,
                lateral_gain=.15, lateral_ki=0., lateral_kd=0., lateral_speed_m_s=0., lateral_enabled=False,
                tag_size_m=.12, target_distance_m=1., vertical_enabled=True, distance_enabled=True)


class VisionControlConfig:
    def __init__(self, path=None):
        self.path = Path(path or os.environ.get('DRONE_VISION_CONTROL_CONFIG') or
                         ROOT / 'config' / 'vision_control.json')

    @staticmethod
    def validate(value):
        optional = {key for key in DEFAULTS if key.startswith('lateral_')}
        if not isinstance(value, dict) or set(value)-set(DEFAULTS) or (set(DEFAULTS)-optional)-set(value):
            raise ValueError('视觉控制参数字段不完整或存在未知字段')
        value = {**DEFAULTS, **value}  # Existing version-1 files remain valid.
        for name, fallback in DEFAULTS.items():
            item = value[name]
            if isinstance(fallback, bool):
                if not isinstance(item, bool):
                    raise ValueError(name + ' 必须是布尔值')
            elif isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
                raise ValueError(name + ' 必须是有限数值')
            elif name in ('tag_size_m', 'target_distance_m') and item <= 0:
                raise ValueError('目标距离和标记边长必须为正数')
            elif 'speed_' in name and item < 0:
                raise ValueError('速度幅值不能为负数')
        return dict(value)

    def load(self):
        try:
            with self.path.open('rb') as stream:
                raw = stream.read(16385)
        except FileNotFoundError:
            return dict(ok=True, stored=False, parameters=dict(DEFAULTS))
        if len(raw) > 16384:
            raise ValueError('视觉默认参数文件过大')
        root = json.loads(raw)
        if not isinstance(root, dict) or root.get('version') != 1:
            raise ValueError('视觉默认参数文件格式无效')
        return dict(ok=True, stored=True, parameters=self.validate(root.get('parameters')))

    def save(self, parameters):
        parameters = self.validate(parameters)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        pending = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix='vision-control-', suffix='.tmp', delete=False) as out:
                pending = Path(out.name)
                json.dump(dict(version=1, parameters=parameters), out, ensure_ascii=False, allow_nan=False, indent=2)
                out.flush()
                os.fsync(out.fileno())
            os.replace(pending, self.path)
            if os.name != 'nt':
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        finally:
            if pending is not None and pending.exists():
                pending.unlink()
        return dict(ok=True, stored=True, parameters=parameters,
                    message='已保存为开发板默认参数，网页和程序重启后继续使用')
