#!/usr/bin/env python3
"""Measure the live two-rover sensor contract and emit machine-readable proof.

The ROS graph retaining a topic name is not evidence that the sensor is
flowing.  This probe subscribes to RGB, RGB-D points, and LiDAR for both
rovers, counts messages for a bounded wall-clock interval, and records useful
message metadata.  ArUco detections are measured too, but are not required: a
camera can be healthy while no marker is in its current field of view.
"""

import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('multirobot_sensor_probe')
    counts = {}
    metadata = {}
    subscriptions = []

    def add(topic, message_type, qos, describe):
        counts[topic] = 0

        def callback(message):
            counts[topic] += 1
            metadata[topic] = describe(message)

        subscriptions.append(node.create_subscription(
            message_type, topic, callback, qos))

    for robot in ('leo1', 'leo2'):
        add(f'/{robot}/scan', LaserScan, qos_profile_sensor_data,
            lambda msg: {
                'frame_id': msg.header.frame_id,
                'samples': len(msg.ranges),
                'range_min_m': msg.range_min,
                'range_max_m': msg.range_max,
            })
        add(f'/{robot}/camera/image', Image, qos_profile_sensor_data,
            lambda msg: {
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
            })
        add(f'/{robot}/camera/camera_info', CameraInfo,
            qos_profile_sensor_data,
            lambda msg: {
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'fx': msg.k[0],
                'fy': msg.k[4],
            })
        add(f'/{robot}/camera/points', PointCloud2, qos_profile_sensor_data,
            lambda msg: {
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'point_step': msg.point_step,
                'row_step': msg.row_step,
                'fields': [field.name for field in msg.fields],
            })
        add(f'/{robot}/tag_detections', MarkerArray, 10,
            lambda msg: {'markers_in_last_message': len(msg.markers)})

    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.monotonic() - started

    required = [
        topic for topic in counts
        if not topic.endswith('/tag_detections')
    ]
    missing = [topic for topic in required if counts[topic] == 0]
    result = {
        'wall_seconds': elapsed,
        'counts': counts,
        'average_hz': {
            topic: round(count / elapsed, 3) for topic, count in counts.items()
        },
        'metadata': metadata,
        'required_topics_missing': missing,
        'pass': not missing,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as handle:
            handle.write(rendered + '\n')

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if not missing else 2)


if __name__ == '__main__':
    main()
