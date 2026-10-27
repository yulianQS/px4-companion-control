"""Bounded ROS web snapshots: fresh frame, stale process, missing and invalid files."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.vision_snapshots import VisionSnapshots


class SnapshotTests(unittest.TestCase):
    def test_fresh_stale_and_invalid(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            reader = VisionSnapshots(p)
            self.assertFalse(reader.status()['camera_ready'])
            status = dict(source='ros2_cpp', written_monotonic=time.monotonic(), camera_ready=True,
                          target_found=True, center=[100, 200], pose_valid=False, actuation_enabled=False)
            (p / 'status.json').write_text(json.dumps(status))
            (p / 'latest.jpg').write_bytes(b'\xff\xd8test\xff\xd9')
            self.assertTrue(reader.status()['target_found'])
            self.assertTrue(reader.frame().startswith(b'\xff\xd8'))
            status['preview_image_ready'] = False
            (p / 'status.json').write_text(json.dumps(status))
            self.assertTrue(reader.status()['target_found'])
            self.assertTrue(reader.status()['camera_ready'])
            status.update(calibrated=True,pose_valid=True,distance_m=1.2,tag_size_m=.12,actuation_enabled=True)
            (p / 'status.json').write_text(json.dumps(status))
            self.assertTrue(reader.status()['calibrated'])
            self.assertTrue(reader.status()['pose_valid'])
            self.assertEqual(reader.status()['distance_m'],1.2)
            self.assertFalse(reader.status()['actuation_enabled'])
            with self.assertRaises(FileNotFoundError):
                reader.frame()
            status['written_monotonic'] -= 1
            (p / 'status.json').write_text(json.dumps(status))
            self.assertFalse(reader.status()['target_found'])
            self.assertEqual(reader.status()['yaw_preview_rad_s'], 0)
            with self.assertRaises(FileNotFoundError):
                reader.frame()
            for invalid in ['[]', '{}', 'NaN', '{']:
                (p / 'status.json').write_text(invalid)
                self.assertFalse(reader.status()['camera_ready'])


if __name__ == '__main__':
    unittest.main()
