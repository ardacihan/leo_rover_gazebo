#!/usr/bin/env python3

"""Fail-closed velocity gate for the physical Leo Rover."""

import math
import time

import numpy
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from exploration_policy import robust_clearance


class SafetyCommandGate(Node):
    """Forward bounded commands only while every safety input is fresh."""

    def __init__(self):
        super().__init__("safety_command_gate")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("battery_topic", "/firmware/battery_averaged")
        self.declare_parameter("cmd_vel_request_topic", "/cmd_vel_request")
        self.declare_parameter("cmd_vel_raw_topic", "/cmd_vel_raw")
        self.declare_parameter("maximum_linear_speed", 0.10)
        self.declare_parameter("maximum_reverse_speed", 0.0)
        self.declare_parameter("maximum_angular_speed", 0.30)
        self.declare_parameter("minimum_battery_voltage", 5.0)
        self.declare_parameter("minimum_finite_scan_points", 30)
        self.declare_parameter("minimum_reverse_clearance", 0.75)
        self.declare_parameter("self_filter_radius", 0.05)
        # The rear corridor below is measured from raw scan angles.  When the
        # LIDAR is not mounted facing forward, this rotates the scan into the
        # base frame.  Rover 4 mounts it backwards (laser_frame yaw = pi), so
        # without this the "rear" check reads the corridor in front.
        self.declare_parameter("scan_yaw_offset", 0.0)
        self.declare_parameter("rear_outlier_points", 5)
        self.declare_parameter("scan_timeout", 0.4)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("battery_timeout", 1.0)
        self.declare_parameter("command_timeout", 0.3)

        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        request_topic = self.get_parameter("cmd_vel_request_topic").value
        raw_topic = self.get_parameter("cmd_vel_raw_topic").value

        if request_topic.rstrip("/") in ("/cmd_vel", "/cmd_vel_raw"):
            raise RuntimeError("request topic would bypass part of the safety chain")
        if raw_topic.rstrip("/") == "/cmd_vel":
            raise RuntimeError("gate output must pass through Collision Monitor")

        self.max_linear = min(
            abs(float(self.get_parameter("maximum_linear_speed").value)), 0.10
        )
        self.max_reverse = min(
            abs(float(self.get_parameter("maximum_reverse_speed").value)), 0.05
        )
        self.max_angular = min(
            abs(float(self.get_parameter("maximum_angular_speed").value)), 0.30
        )
        self.minimum_battery = max(
            float(self.get_parameter("minimum_battery_voltage").value),
            5.0,
        )
        self.minimum_finite_points = max(
            int(self.get_parameter("minimum_finite_scan_points").value), 1
        )
        self.minimum_reverse_clearance = min(
            max(
                float(self.get_parameter("minimum_reverse_clearance").value),
                0.50,
            ),
            1.50,
        )
        self.self_filter_radius = min(
            max(float(self.get_parameter("self_filter_radius").value), 0.0),
            0.15,
        )
        self.scan_yaw_offset = math.atan2(
            math.sin(float(self.get_parameter("scan_yaw_offset").value)),
            math.cos(float(self.get_parameter("scan_yaw_offset").value)),
        )
        self.rear_outlier_points = min(
            max(int(self.get_parameter("rear_outlier_points").value), 0),
            10,
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.battery_timeout = float(
            self.get_parameter("battery_timeout").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )

        self.publisher = self.create_publisher(Twist, raw_topic, 10)
        self.create_subscription(
            Twist, request_topic, self._command_callback, 10
        )
        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, battery_topic, self._battery_callback, qos_profile_sensor_data
        )

        self.command = Twist()
        self.command_time = None
        self.scan_time = None
        self.odom_time = None
        self.battery_time = None
        self.battery = None
        self.finite_scan_points = 0
        self.rear_clearance = None
        self._scan_geometry = None
        self._cos = None
        self._sin = None
        self.last_reason = None
        self.odom_messages = 0
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            "fail-closed command gate ready: "
            f"linear_limit={self.max_linear:.2f} m/s, "
            f"reverse_limit={self.max_reverse:.2f} m/s, "
            f"angular_limit={self.max_angular:.2f} rad/s, "
            f"battery_min={self.minimum_battery:.2f} V"
        )

    def _command_callback(self, msg):
        self.command = msg
        self.command_time = time.monotonic()

    def _scan_callback(self, msg):
        # Vectorised because the per-ray Python loop this replaces consumed
        # enough CPU to delay /scan delivery to Collision Monitor, which then
        # discarded the source and stopped constraining motion.
        count = len(msg.ranges)
        key = (msg.angle_min, msg.angle_increment, count)
        if key != self._scan_geometry:
            angles = (
                float(msg.angle_min)
                + self.scan_yaw_offset
                + numpy.arange(count, dtype=float) * float(msg.angle_increment)
            )
            self._cos = numpy.cos(angles)
            self._sin = numpy.sin(angles)
            self._scan_geometry = key

        ranges = numpy.asarray(msg.ranges, dtype=float)
        valid = (
            numpy.isfinite(ranges)
            & (ranges >= float(msg.range_min))
            & (ranges <= float(msg.range_max))
        )
        self.finite_scan_points = int(numpy.count_nonzero(valid))

        usable = valid & (ranges >= self.self_filter_radius)
        x = ranges * self._cos
        y = ranges * self._sin
        rear = usable & (x < 0.0) & (numpy.abs(y) <= 0.30)

        self.rear_clearance = robust_clearance(
            (-x[rear]).tolist(),
            self.rear_outlier_points,
            default_clearance=float(msg.range_max),
        )
        self.scan_time = time.monotonic()

    def _odom_callback(self, _msg):
        self.odom_messages += 1
        self.odom_time = time.monotonic()

    def _battery_callback(self, msg):
        self.battery = float(msg.data)
        self.battery_time = time.monotonic()

    @staticmethod
    def _fresh(timestamp, timeout, now):
        return timestamp is not None and now - timestamp <= timeout

    def _blocking_reason(self, now):
        if not self._fresh(self.command_time, self.command_timeout, now):
            return "stale command"
        if not self._fresh(self.scan_time, self.scan_timeout, now):
            return "stale lidar scan"
        if self.finite_scan_points < self.minimum_finite_points:
            return f"only {self.finite_scan_points} finite scan points"
        if not self._fresh(self.odom_time, self.odom_timeout, now):
            return "stale wheel odometry"
        if not self._fresh(self.battery_time, self.battery_timeout, now):
            return "stale battery telemetry"
        if self.battery is None or self.battery < self.minimum_battery:
            voltage = "unknown" if self.battery is None else f"{self.battery:.2f} V"
            return f"low battery ({voltage})"
        requested_linear = float(self.command.linear.x)
        if requested_linear < -0.001:
            if self.max_reverse <= 0.0:
                return "reverse motion is disabled"
            if abs(float(self.command.angular.z)) > 0.01:
                return "reverse command must be straight"
            if (
                self.rear_clearance is None
                or self.rear_clearance < self.minimum_reverse_clearance
            ):
                clearance = (
                    "unknown"
                    if self.rear_clearance is None
                    else f"{self.rear_clearance:.2f} m"
                )
                return (
                    "rear corridor blocked "
                    f"({clearance} < {self.minimum_reverse_clearance:.2f} m)"
                )
        return None

    def _timer_callback(self):
        now = time.monotonic()
        reason = self._blocking_reason(now)
        output = Twist()
        if reason is None:
            requested_linear = float(self.command.linear.x)
            output.linear.x = min(
                max(requested_linear, -self.max_reverse), self.max_linear
            )
            if output.linear.x < -0.001:
                output.angular.z = 0.0
            else:
                output.angular.z = min(
                    max(float(self.command.angular.z), -self.max_angular),
                    self.max_angular,
                )
        self.publisher.publish(output)

        if reason != self.last_reason:
            odom_age = (
                "never"
                if self.odom_time is None
                else f"{now - self.odom_time:.3f}s/{self.odom_messages} msgs"
            )
            if reason is None:
                self.get_logger().info(
                    f"gate open: all safety inputs are fresh; odom={odom_age}"
                )
            else:
                self.get_logger().warn(
                    f"gate closed: {reason}; odom={odom_age}"
                )
            self.last_reason = reason

    def publish_final_zeros(self):
        zero = Twist()
        for _ in range(20):
            self.publisher.publish(zero)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyCommandGate()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_final_zeros()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
