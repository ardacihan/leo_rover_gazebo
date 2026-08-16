#!/usr/bin/env python3
"""Short ROS topic liveness probe that does not depend on ros2cli daemon."""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan


TOPICS = {
    '/clock': Clock,
    '/leo1/scan': LaserScan,
    '/leo1/camera/image': Image,
    '/leo1/odom': Odometry,
}


def main():
    rclpy.init()
    node = rclpy.create_node('validation_topic_probe')
    counts = {topic: 0 for topic in TOPICS}
    subscriptions = []
    for topic, message_type in TOPICS.items():
        subscriptions.append(node.create_subscription(
            message_type, topic,
            lambda _message, name=topic: counts.__setitem__(
                name, counts[name] + 1), 10))
    started = time.monotonic()
    while time.monotonic() - started < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(json.dumps({'wall_seconds': 10, 'counts': counts}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if any(value == 0 for value in counts.values()):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
