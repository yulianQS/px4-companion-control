#!/usr/bin/env python3
"""Short real camera test; stops its own capture and checks loss, never opens flight UART."""
import json
import argparse
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grayscale', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    binaries = root / 'ros2_ws/install/drone_vision/lib/drone_vision'
    rclpy.init()
    node = rclpy.create_node('camera_pipeline_test')
    result = {'images': 0, 'info': None}

    def image(msg):
        assert msg.encoding == ('mono8' if args.grayscale else 'bgr8') and msg.width == 640 and msg.height == 480
        assert msg.step == (640 if args.grayscale else 1920) and len(msg.data) == msg.step * msg.height
        result['images'] += 1

    image_sub = node.create_subscription(Image, '/camera/image_raw', image, qos_profile_sensor_data)
    info_sub = node.create_subscription(CameraInfo, '/camera/camera_info',
        lambda msg: result.update(info=msg), qos_profile_sensor_data)
    children = []
    with tempfile.TemporaryDirectory(prefix='ros2-camera-test-') as temp:
        temp = Path(temp)
        log = (temp / 'nodes.log').open('wb')
        try:
            for exe, options in [('camera_node', ['-p', 'grayscale:=' + str(args.grayscale).lower()]), ('tag_detector_node', []),
                                 ('follow_preview_node', ['-p', 'snapshot_directory:=' + str(temp / 'snapshot')])]:
                children.append(subprocess.Popen([str(binaries / exe), '--ros-args'] + options,
                                                  stdout=log, stderr=log))
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                status = temp / 'snapshot/status.json'
                if result['images'] >= 3 and status.exists() and json.loads(status.read_text()).get('camera_ready'):
                    break
                assert all(p.poll() is None for p in children), (temp / 'nodes.log').read_text(errors='replace')
            else:
                raise AssertionError('Camera pipeline timeout: ' + str(result) + '\n' + (temp / 'nodes.log').read_text(errors='replace')[-6000:])
            assert result['info'] is not None and result['info'].k[0] == 0
            status = json.loads((temp / 'snapshot/status.json').read_text())
            assert not status['pose_valid'] and status['yaw_preview_rad_s'] == 0
            assert not status['camera_upright_confirmed'] and not status['actuation_enabled']
            (Path('/tmp') / 'drone_ros2_camera_sample_20261011.jpg').write_bytes((temp / 'snapshot/latest.jpg').read_bytes())
            children[0].terminate()
            children[0].wait(timeout=8)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                status = json.loads((temp / 'snapshot/status.json').read_text())
                if not status['camera_ready'] and not status['target_found'] and status['yaw_preview_rad_s'] == 0:
                    break
            else:
                raise AssertionError('Camera stop did not invalidate state')
            print('PASS: real IMX219 NV12→' + ('MONO8' if args.grayscale else 'BGR8') + '→ROS→JPEG; uncalibrated CameraInfo; default zero preview; capture stop invalidates state; frames=' + str(result['images']))
        finally:
            for child in children:
                if child.poll() is None:
                    child.terminate()
            for child in children:
                child.wait(timeout=8)
            log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
