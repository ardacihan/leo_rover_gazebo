#!/usr/bin/env python3

"""Firmware odometry and command relay, without owning the odom transform.

`leo_nav_bridge` does three jobs at once: republish firmware wheel odometry,
relay `/cmd_vel` to the namespaced firmware topic, and broadcast
`odom -> base_footprint`. It hard-codes that broadcast with no way to disable
it, so an EKF cannot own the transform while it runs.

This relay does the first two jobs and makes the transform optional, so
`leo-nav-bridge.service` can be stopped and robot_localization can fuse wheel
odometry with the gyro instead. Behaviour and covariances match the original
so the swap changes nothing else.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from leo_msgs.msg import WheelOdom
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


class FirmwareRelay(Node):
    """Republish wheel odometry and forward velocity commands."""

    def __init__(self):
        super().__init__("firmware_relay")
        self.declare_parameter("wheel_odom_topic", "/rob_2/firmware/wheel_odom")
        self.declare_parameter("firmware_cmd_topic", "/rob_2/cmd_vel")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom_tf", False)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_odom_tf = bool(
            self.get_parameter("publish_odom_tf").value
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.wheel_odom_pub = self.create_publisher(
            Odometry, "/wheel_odom", sensor_qos
        )
        self.merged_odom_pub = self.create_publisher(Odometry, "/merged_odom", 10)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("firmware_cmd_topic").value), 10
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_odom_tf else None
        )
        self.create_subscription(
            WheelOdom, str(self.get_parameter("wheel_odom_topic").value),
            self._wheel_odom_callback, sensor_qos
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value),
            self._cmd_vel_callback, 10
        )
        self.get_logger().info(
            "firmware relay ready: wheel odom -> /wheel_odom, /merged_odom; "
            f"cmd_vel -> firmware; odom TF "
            f"{'published here' if self.publish_odom_tf else 'left to the EKF'}"
        )

    def _wheel_odom_callback(self, msg):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(msg.pose_x)
        odom.pose.pose.position.y = float(msg.pose_y)
        half_yaw = float(msg.pose_yaw) * 0.5
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x = float(msg.velocity_lin)
        odom.twist.twist.angular.z = float(msg.velocity_ang)
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[35] = 0.001
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[35] = 0.001
        self.wheel_odom_pub.publish(odom)
        self.merged_odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _cmd_vel_callback(self, msg):
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FirmwareRelay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
