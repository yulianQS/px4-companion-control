#!/usr/bin/env python3
"""C++ ROS graph integration on synthetic images, never opens camera or flight UART."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from drone_vision.msg import Target2D


def run():
    root = Path(__file__).resolve().parents[1]
    binaries = root / 'ros2_ws/install/drone_vision/lib/drone_vision'
    build = root / 'ros2_ws/build/drone_vision'
    rclpy.init()
    node = rclpy.create_node('vision_pipeline_test')
    received = {'target': None, 'command': None}
    target_sub = node.create_subscription(Target2D, '/vision/target_2d',
        lambda msg: received.update(target=msg), qos_profile_sensor_data)
    command_sub = node.create_subscription(TwistStamped, '/follow/cmd_preview',
        lambda msg: received.update(command=msg), 1)
    phase_pub = node.create_publisher(String, '/_vision_test/phase', 1)
    processes = []
    with tempfile.TemporaryDirectory(prefix='ros2-vision-test-') as temp:
        temp = Path(temp)
        subprocess.run([str(build / 'vision_algorithms_test'), str(temp / 'tag.bgr')], check=True)
        log = (temp / 'nodes.log').open('wb')
        try:
            for exe, options in [(binaries / 'tag_detector_node', ['-p', 'denoise_sigma:=0.8']),
                    (binaries / 'follow_preview_node', ['-p', 'camera_upright_confirmed:=true',
                         '-p', 'snapshot_directory:=' + str(temp / 'snapshots')]),
                    (build / 'vision_test_image_source', ['-p', 'image_path:=' + str(temp / 'tag.bgr')])]:
                processes.append(subprocess.Popen([str(exe), '--ros-args'] + options, stdout=log, stderr=log))

            def drive(condition, phase, seconds=8):
                deadline = time.monotonic() + seconds
                next_phase = 0
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_phase:
                        phase_pub.publish(String(data=phase))
                        next_phase = time.monotonic() + .2
                    rclpy.spin_once(node, timeout_sec=.08)
                    if condition():
                        return
                    assert all(p.poll() is None for p in processes), (temp / 'nodes.log').read_text(errors='replace')
                raise AssertionError('ROS pipeline timeout: ' + str(received) + '\n' + (temp / 'nodes.log').read_text(errors='replace'))

            drive(lambda: received['target'] is not None and received['target'].valid
                  and received['command'] is not None and received['command'].twist.angular.z > 0, 'tag')
            command = received['command']
            received['target'] = None
            drive(lambda: received['target'] is not None and received['target'].valid
                  and abs(received['target'].center_u - 429.5) < 2, 'mono')
            drive(lambda: received['target'].reason == 'Invalid/stale MONO8 image', 'stale_mono')
            drive(lambda: received['target'].valid, 'tag')
            print('PASS: MONO8 matches BGR8 pixel coordinates; stale MONO8 rejected')
            assert command.header.frame_id == 'base_link'
            assert command.twist.linear.x == command.twist.linear.y == command.twist.linear.z == 0
            # Runtime preview settings: atomic apply, stop, limits, invalid-file fail-closed and defaults restore.
            snapshot = temp / 'snapshots/status.json'
            config = temp / 'snapshots/preview_config.json'
            initial = json.loads(snapshot.read_text())['preview_parameters']
            def configure(revision, parameters):
                pending = config.with_suffix('.tmp')
                pending.write_text(json.dumps(dict(revision=revision, parameters=parameters)))
                pending.replace(config)
            stopped = dict(initial, preview_enabled=False)
            configure('test-disabled', stopped)
            drive(lambda: json.loads(snapshot.read_text()).get('preview_config_revision') == 'test-disabled'
                  and received['command'].twist.angular.z == 0, 'tag')
            active = dict(initial, preview_enabled=True, yaw_gain=2., max_yaw_rate=.1)
            configure('test-limit', active)
            drive(lambda: json.loads(snapshot.read_text()).get('preview_config_revision') == 'test-limit'
                  and abs(received['command'].twist.angular.z - (429.5-319.5)/320*2) < 1e-6, 'tag')
            configure('test-invalid', dict(active, yaw_gain='invalid'))
            drive(lambda: json.loads(snapshot.read_text()).get('preview_enabled') is False
                  and received['command'].twist.angular.z == 0, 'tag')
            config.unlink()
            drive(lambda: json.loads(snapshot.read_text()).get('preview_parameters') == initial
                  and received['command'].twist.angular.z > 0, 'tag')
            print('PASS: preview config readback/disable/direct gain; malformed config fails closed; YAML defaults restore')
            drive(lambda: received['target'] is not None and not received['target'].valid
                  and received['command'].twist.angular.z == 0, 'stop', seconds=3)
            drive(lambda: received['target'].reason == 'Invalid/stale BGR8 image'
                  and received['command'].twist.angular.z == 0, 'stale')
            drive(lambda: received['target'].reason == 'target_not_found_or_ambiguous'
                  and received['command'].twist.angular.z == 0, 'blank')
            # Stop source and detector, then directly inject a malformed small target message.
            for index in [2, 0]:
                processes[index].terminate()
                processes[index].wait(timeout=5)
            processes = [processes[1]]
            target_pub = node.create_publisher(Target2D, '/vision/target_2d', qos_profile_sensor_data)
            # Fresh detections without ANY JPEG publisher/viewer must remain usable by the controller.
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                good = Target2D()
                good.header.stamp = node.get_clock().now().to_msg()
                good.header.frame_id = 'camera_optical_frame'
                good.valid, good.tag_id = True, 0
                good.image_width, good.image_height = 640, 480
                good.center_u, good.center_v, good.area_px2 = 400., 200., 400.
                good.corners_uv = [390.,190.,410.,190.,410.,210.,390.,210.]
                target_pub.publish(good)
                rclpy.spin_once(node, timeout_sec=.08)
                state = json.loads(snapshot.read_text())
                if state['target_found'] and state['camera_ready'] and not state['preview_image_ready']:
                    break
            else:
                raise AssertionError('Fresh target incorrectly depends on JPEG delivery')
            print('PASS: backend target remains valid without JPEG publisher or browser')
            time.sleep(.1)
            before=json.loads(snapshot.read_text())
            time.sleep(.12)
            after=json.loads(snapshot.read_text())
            assert before['observation_monotonic']==after['observation_monotonic']
            assert after['written_monotonic']>before['written_monotonic']
            print('PASS: snapshot timer does not manufacture new detection timestamps')
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                bad = Target2D()
                bad.header.stamp = node.get_clock().now().to_msg()
                bad.header.frame_id = 'camera_optical_frame'
                bad.valid, bad.image_width, bad.image_height = True, 640, 480
                bad.center_u, bad.center_v, bad.area_px2 = math.nan, 100., 25000.
                target_pub.publish(bad)
                rclpy.spin_once(node, timeout_sec=.1)
                state = json.loads((temp / 'snapshots/status.json').read_text())
                if state['message'] == 'invalid_target_measurement':
                    break
            else:
                raise AssertionError('Malformed measurement not delivered/rejected')
            assert not state['target_found'] and not state['pose_valid'] and not state['actuation_enabled']
            assert state['yaw_preview_rad_s'] == 0
            print('PASS: C++ image→AprilTag→yaw preview; zero translation; loss/stale/blank/NaN invalidation; no metric pose')
        finally:
            for p in processes:
                if p.poll() is None:
                    p.terminate()
            for p in processes:
                p.wait(timeout=8)
            log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
