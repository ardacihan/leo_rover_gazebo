#!/usr/bin/env python3
"""Capture N raw LaserScan messages to JSON so degradation can be reviewed offline.

Usage (inside the sim container):
    python3 /ros2_ws/scripts/capture_scan_sample.py <out.json> [count]
"""

import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Capture(Node):

    def __init__(self, path, count):
        super().__init__('capture_scan_sample')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.path = path
        self.count = count
        self.scans = []
        self.create_subscription(LaserScan, '/leo1/scan', self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg):
        if len(self.scans) >= self.count:
            return
        self.scans.append({
            'frame_id': msg.header.frame_id,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': [None if not (r == r) or r in (float('inf'),) else float(r)
                       for r in msg.ranges],
        })
        self.get_logger().info(f'captured {len(self.scans)}/{self.count}')


def main():
    path = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    rclpy.init()
    node = Capture(path, count)
    while rclpy.ok() and len(node.scans) < count:
        rclpy.spin_once(node, timeout_sec=0.5)
    with open(path, 'w') as fh:
        json.dump(node.scans, fh)
    print(f'wrote {len(node.scans)} scans to {path}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
