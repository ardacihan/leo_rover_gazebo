#!/usr/bin/env python3

"""Integrate wheel twist without the stationary IMU yaw drift."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class WheelOdomTf(Node):
    """Publish wheel-only odometry and the odom-to-base transform."""

    def __init__(self):
        super().__init__("wheel_odom_tf")
        self.declare_parameter("input_topic", "/wheel_odom")
        self.declare_parameter("output_topic", "/wheel_odom_integrated")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("linear_deadband", 0.005)
        self.declare_parameter("angular_deadband", 0.01)
        self.declare_parameter("maximum_dt", 0.20)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.linear_deadband = abs(
            float(self.get_parameter("linear_deadband").value)
        )
        self.angular_deadband = abs(
            float(self.get_parameter("angular_deadband").value)
        )
        self.maximum_dt = max(float(self.get_parameter("maximum_dt").value), 0.05)

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.last_stamp_ns = None
        self.last_input_time = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.timer = self.create_timer(0.02, self._publish_state)
        self.get_logger().info(
            f"integrating wheel odometry: {input_topic} -> {output_topic}, "
            f"TF {self.odom_frame} -> {self.base_frame}"
        )

    @staticmethod
    def _quaternion_from_yaw(yaw):
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    def _odom_callback(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        linear = float(msg.twist.twist.linear.x)
        angular = float(msg.twist.twist.angular.z)
        if abs(linear) < self.linear_deadband:
            linear = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0

        if self.last_stamp_ns is not None:
            dt = (stamp_ns - self.last_stamp_ns) / 1_000_000_000.0
            if 0.0 < dt <= self.maximum_dt:
                midpoint_yaw = self.yaw + 0.5 * angular * dt
                self.x += linear * math.cos(midpoint_yaw) * dt
                self.y += linear * math.sin(midpoint_yaw) * dt
                self.yaw = math.atan2(
                    math.sin(self.yaw + angular * dt),
                    math.cos(self.yaw + angular * dt),
                )
        self.last_stamp_ns = stamp_ns
        self.last_input_time = time.monotonic()
        self.linear_velocity = linear
        self.angular_velocity = angular

    def _publish_state(self):
        # Stamp at publication time so scan consumers have a current transform
        # even when the firmware's 20 Hz odometry arrives in short bursts.
        stamp = self.get_clock().now().to_msg()
        input_is_fresh = (
            self.last_input_time is not None
            and time.monotonic() - self.last_input_time <= self.maximum_dt
        )
        linear = self.linear_velocity if input_is_fresh else 0.0
        angular = self.angular_velocity if input_is_fresh else 0.0

        quat_z, quat_w = self._quaternion_from_yaw(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[35] = 0.001
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdomTf()
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
