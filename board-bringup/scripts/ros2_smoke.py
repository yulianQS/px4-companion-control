#!/usr/bin/python3
"""Two-process localhost DDS delivery check. No flight-controller connection."""
import argparse
import time
import rclpy
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('role', choices=['publisher', 'subscriber'])
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('drone_check_' + args.role)
received = []
try:
    if args.role == 'subscriber':
        sub = node.create_subscription(String, '/drone_check/probe', lambda msg: received.append(msg.data), 1)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and 'drone_check_ok' not in received:
            rclpy.spin_once(node, timeout_sec=0.2)
        if 'drone_check_ok' not in received:
            raise RuntimeError('DDS_DELIVERY_FAILED')
        print('DDS_DELIVERY_OK', flush=True)
    else:
        pub = node.create_publisher(String, '/drone_check/probe', 1)
        message = String(data='drone_check_ok')
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.2)
        print('PUBLISHER_DONE', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
