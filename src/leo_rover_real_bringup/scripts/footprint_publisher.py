#!/usr/bin/env python3
"""Publish the rover's static collision footprint for Nav2 Collision Monitor.

The Humble Collision Monitor's approach-action polygon has no static points
parameter; it only accepts a footprint topic. This node publishes the chassis
rectangle (0.44 x 0.44 m plus margin) so the approach check simulates the true
footprint instead of the old 0.35 m circumscribed circle, which vetoed motion
past obstacles that were laterally clear of the wheels (a table leg 8 cm from
the left wheel held the rover for 40 s on 2026-08-14).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, PolygonStamped


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__("footprint_publisher")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("footprint_topic", "/collision_monitor/footprint")
        # Chassis is +/-0.22 m; margins: +0.06 front, +0.04 sides and rear.
        self.declare_parameter("front", 0.28)
        self.declare_parameter("rear", 0.26)
        self.declare_parameter("half_width", 0.26)
        self.declare_parameter("rate", 2.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        front = float(self.get_parameter("front").value)
        rear = float(self.get_parameter("rear").value)
        half_width = float(self.get_parameter("half_width").value)
        self.corners = [
            (front, half_width),
            (front, -half_width),
            (-rear, -half_width),
            (-rear, half_width),
        ]
        self.publisher = self.create_publisher(
            PolygonStamped,
            str(self.get_parameter("footprint_topic").value),
            1,
        )
        period = 1.0 / max(float(self.get_parameter("rate").value), 0.1)
        self.timer = self.create_timer(period, self._publish)

    def _publish(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in self.corners
        ]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
