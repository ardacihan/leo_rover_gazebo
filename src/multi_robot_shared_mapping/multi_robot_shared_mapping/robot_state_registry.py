#!/usr/bin/env python3
"""
Shared robot-state registry.

This is intentionally small and extendable:
- now: stores /leo1/odom and /leo2/odom;
- later: add ArUco detections, assigned exploration zones, frontier goals.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class RobotStateRegistry(Node):
    def __init__(self):
        super().__init__("robot_state_registry")

        self.robots = {}

        self.create_subscription(Odometry, "/leo1/odom", lambda msg: self.odom_cb("leo1", msg), 10)
        self.create_subscription(Odometry, "/leo2/odom", lambda msg: self.odom_cb("leo2", msg), 10)

        self.pub = self.create_publisher(String, "/shared/robot_states", 10)
        self.timer = self.create_timer(1.0, self.publish_state)

    def odom_cb(self, robot_name: str, msg: Odometry):
        p = msg.pose.pose.position
        self.robots[robot_name] = {
            "x": p.x,
            "y": p.y,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
        }

    def publish_state(self):
        lines = []
        for robot_name, state in sorted(self.robots.items()):
            lines.append(f"{robot_name}: x={state['x']:.2f}, y={state['y']:.2f}")

        if "leo1" in self.robots and "leo2" in self.robots:
            x1, y1 = self.robots["leo1"]["x"], self.robots["leo1"]["y"]
            x2, y2 = self.robots["leo2"]["x"], self.robots["leo2"]["y"]
            dist = math.hypot(x1 - x2, y1 - y2)
            lines.append(f"distance(leo1, leo2)={dist:.2f} m")

        msg = String()
        msg.data = "\n".join(lines)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateRegistry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
