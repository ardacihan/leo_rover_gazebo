#!/usr/bin/env python3
"""Relay /firmware/wheel_odom -> /wheel_odom (rover 2 has no leo_nav_bridge).

The nav stack (nav2 odom_topic, velocity guard, preflight) expects root
/wheel_odom; rover 2's firmware publishes /firmware/wheel_odom. Pass the
message through unmodified -- TF freshness is tf_freshener.py's job.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomRelay(Node):
    def __init__(self):
        super().__init__("wheel_odom_relay")
        self.pub = self.create_publisher(
            Odometry, "/wheel_odom", qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/firmware/wheel_odom", self.pub.publish,
            qos_profile_sensor_data,
        )
        self.get_logger().info("/firmware/wheel_odom -> /wheel_odom")


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
