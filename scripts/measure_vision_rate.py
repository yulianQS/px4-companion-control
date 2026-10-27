#!/usr/bin/env python3
"""Passive ROS vision throughput check. Never starts camera/control or sends flight commands."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from drone_vision.msg import Target2D
from log_storage import session_directory, utc, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    output = session_directory('performance', 'vision-rate')
    rclpy.init()
    node = rclpy.create_node('vision_rate_measurement')
    counts = dict(processed=0, found=0, pose=0, preview=0, rejected=0)
    ages = []
    last_stamp = None

    def target(msg):
        nonlocal last_stamp
        if msg.reason not in ('target_detected', 'target_not_found_or_ambiguous'):
            counts['rejected'] += 1
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == last_stamp:
            return
        last_stamp = stamp
        counts['processed'] += 1
        counts['found'] += int(msg.valid)
        counts['pose'] += int(msg.pose_valid)
        ages.append((node.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000)

    def preview(msg):
        counts['preview'] += 1

    subscriptions = [
        node.create_subscription(Target2D, '/vision/target_2d', target, qos_profile_sensor_data),
        node.create_subscription(CompressedImage, '/vision/annotated/compressed', preview, qos_profile_sensor_data),
    ]
    started_utc = utc()
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        seconds = time.monotonic() - start
        ordered = sorted(ages)
        result = dict(started_utc=started_utc, seconds=seconds, counts=counts,
                      hz={k: v / seconds for k, v in counts.items()},
                      mean_age_ms=sum(ages) / len(ages) if ages else None,
                      p95_age_ms=ordered[int(.95 * (len(ordered) - 1))] if ordered else None,
                      vision_config=(Path(__file__).resolve().parents[1] / 'config/vision.yaml').read_text())
        write_json(output / 'summary.json', result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(output / 'summary.json')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
