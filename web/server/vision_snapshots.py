"""Read bounded latest C++ ROS snapshots; never connect to a flight-control interface."""
import json
import math
import time
from pathlib import Path


class VisionSnapshots:
    def __init__(self, directory='/dev/shm/drone_vision', max_age=0.5):
        self.directory = Path(directory)
        self.max_age = max_age

    @staticmethod
    def unavailable(message):
        return dict(source='ros2_cpp', camera_ready=False, target_found=False, center=None,
                    pose_valid=False, calibrated=False, actuation_enabled=False,
                    yaw_preview_rad_s=0, linear_preview_m_s=[0, 0, 0], message=message)

    def status(self):
        try:
            with (self.directory / 'status.json').open('rb') as stream:
                raw = stream.read(65537)
            if len(raw) > 65536:
                raise ValueError('snapshot too large')
            value = json.loads(raw)
            if not isinstance(value, dict) or value.get('source') != 'ros2_cpp':
                raise ValueError('invalid snapshot source')
            stamp = value.get('written_monotonic')
            if isinstance(stamp, bool) or not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
                raise ValueError('invalid timestamp')
            age = time.monotonic() - stamp
            if not 0 <= age < self.max_age:
                return self.unavailable('ROS2视觉状态已过期或节点已停止')
            value['snapshot_age_s'] = age
            # ROS supplies calibrated metric pose; this HTTP adapter never actuates.
            value['actuation_enabled'] = False
            return value
        except (OSError, ValueError, TypeError):
            return self.unavailable('等待ROS2 C++视觉节点')

    def frame(self):
        state = self.status()
        if state.get('preview_image_ready', state.get('camera_ready')) is not True:
            raise FileNotFoundError('camera frame unavailable or stale')
        with (self.directory / 'latest.jpg').open('rb') as stream:
            data = stream.read(2 * 1024 * 1024 + 1)
        if len(data) > 2 * 1024 * 1024 or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
            raise ValueError('invalid JPEG')
        return data


class VisionPreviewConfig:
    """Atomic preview-only settings; no ROS subprocess, flight connection or system configuration."""
    BOUNDS = {'yaw_gain': (-math.inf, math.inf), 'max_yaw_rate': (0, 0.5),
              'deadzone_normalized': (0, 0.5), 'max_target_age_seconds': (0.1, 1)}
    FLAGS = {'camera_upright_confirmed', 'preview_enabled'}

    def __init__(self, snapshots):
        self.snapshots = snapshots

    def apply(self, parameters):
        import uuid
        import tempfile
        import os
        if not isinstance(parameters, dict) or set(parameters) != set(self.BOUNDS) | self.FLAGS:
            raise ValueError('视觉参数字段不完整或存在未知字段')
        for name, (low, high) in self.BOUNDS.items():
            value = parameters[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('视觉参数超出范围：' + name)
        if any(not isinstance(parameters[name], bool) for name in self.FLAGS):
            raise ValueError('视觉使能和方向确认必须是布尔值')
        # Require the live node's settings contract, rather than accepting a write into an absent service.
        state = self.snapshots.status()
        if 'preview_parameters' not in state:
            raise ValueError('视觉建议节点未就绪或版本不支持参数应用')
        revision = uuid.uuid4().hex
        directory = self.snapshots.directory
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=directory, prefix='preview-config-', suffix='.tmp', delete=False) as out:
                name = out.name
                json.dump({'revision': revision, 'parameters': parameters}, out, allow_nan=False)
            os.replace(name, directory / 'preview_config.json')
        finally:
            if name and os.path.exists(name):
                os.unlink(name)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            state = self.snapshots.status()
            if state.get('preview_config_revision') == revision and state.get('preview_parameters') == parameters:
                return dict(ok=True, applied=True, revision=revision, vision=state, message='机载C++已应用并回读视觉建议参数；未发送飞控')
            time.sleep(0.05)
        return dict(ok=True, applied=False, revision=revision, message='参数已提交，等待C++节点回读；尚未确认生效')
