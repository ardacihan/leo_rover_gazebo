"""ROS wrapper around the fail-closed velocity guard logic."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan

from .velocity_guard_logic import GuardConfig, GuardState, evaluate_guard, min_range_in_cone


class VelocityGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_guard")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel_guarded")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("require_battery", False)
        self.declare_parameter("require_camera", False)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("command_timeout", 0.60)
        self.declare_parameter("scan_timeout", 0.60)
        self.declare_parameter("odom_timeout", 1.00)
        self.declare_parameter("battery_timeout", 2.00)
        self.declare_parameter("min_battery_voltage", 11.0)
        self.declare_parameter("minimum_valid_scan_points", 30)
        self.declare_parameter("front_stop_distance", 0.0)
        self.declare_parameter("stop_half_angle", 0.70)
        self.declare_parameter("stop_min_points", 3)
        self.declare_parameter("blocked_turn_speed", 0.35)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("diagnostic_rate", 1.0)

        if bool(self.get_parameter("require_camera").value):
            raise RuntimeError(
                "require_camera=true is intentionally unsupported: camera data belongs in the "
                "optional local VoxelLayer and must not gate LiDAR navigation."
            )

        self._config = GuardConfig(
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            command_timeout=float(self.get_parameter("command_timeout").value),
            scan_timeout=float(self.get_parameter("scan_timeout").value),
            odom_timeout=float(self.get_parameter("odom_timeout").value),
            require_battery=bool(self.get_parameter("require_battery").value),
            battery_timeout=float(self.get_parameter("battery_timeout").value),
            min_battery_voltage=float(self.get_parameter("min_battery_voltage").value),
            minimum_valid_scan_points=int(
                self.get_parameter("minimum_valid_scan_points").value
            ),
            front_stop_distance=float(self.get_parameter("front_stop_distance").value),
            stop_half_angle=float(self.get_parameter("stop_half_angle").value),
            stop_min_points=int(self.get_parameter("stop_min_points").value),
            blocked_turn_speed=float(self.get_parameter("blocked_turn_speed").value),
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        battery_topic = str(self.get_parameter("battery_topic").value)

        self._requested = Twist()
        self._command_stamp: Optional[float] = None
        self._scan_stamp: Optional[float] = None
        self._odom_stamp: Optional[float] = None
        self._battery_stamp: Optional[float] = None
        self._battery_voltage: Optional[float] = None
        self._last_reason: Optional[str] = None
        self._invalid_scan_count = 0
        self._scan_valid_points: Optional[int] = None
        self._front_min_range: Optional[float] = None
        self._front_hit_points = 0
        self._rear_min_range: Optional[float] = None
        self._rear_hit_points = 0
        self._left_min_range: Optional[float] = None
        self._right_min_range: Optional[float] = None

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._command_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)
        # Sensor-data QoS, like the scan: the rover's /wheel_odom is published
        # best-effort, and a reliable subscription is incompatible with it --
        # the guard then sees odometry as permanently stale and zeroes every
        # command (found replaying drive_2026-08-20 through the stack).
        self.create_subscription(Odometry, odom_topic, self._odom_callback,
                                 qos_profile_sensor_data)
        if self._config.require_battery:
            self.create_subscription(BatteryState, battery_topic, self._battery_callback, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.create_timer(1.0 / publish_rate, self._publish_guarded_command)
        self.get_logger().info(
            f"Guarding {input_topic} -> {output_topic}; scan={scan_topic}, odom={odom_topic}, "
            f"battery_required={self._config.require_battery}"
        )

    def _command_callback(self, message: Twist) -> None:
        self._requested = message
        self._command_stamp = time.monotonic()

    def _scan_callback(self, message: LaserScan) -> None:
        valid_points = sum(
            1
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        )
        self._scan_stamp = time.monotonic()
        self._scan_valid_points = valid_points
        half = self._config.stop_half_angle
        common = dict(
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            half_angle=half,
        )
        self._front_min_range, self._front_hit_points = min_range_in_cone(
            message.ranges, center=0.0, **common
        )
        self._rear_min_range, self._rear_hit_points = min_range_in_cone(
            message.ranges, center=math.pi, **common
        )
        self._left_min_range, _ = min_range_in_cone(
            message.ranges, center=math.pi / 2.0, **common
        )
        self._right_min_range, _ = min_range_in_cone(
            message.ranges, center=-math.pi / 2.0, **common
        )
        if valid_points >= self._config.minimum_valid_scan_points:
            self._invalid_scan_count = 0
        else:
            self._invalid_scan_count += 1
            if self._invalid_scan_count in (1, 10, 50):
                self.get_logger().warning(
                    "LaserScan has only "
                    f"{valid_points} finite in-range samples; "
                    f"need {self._config.minimum_valid_scan_points} "
                    f"({self._invalid_scan_count} consecutive)"
                )

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.angular.z,
        )
        if all(math.isfinite(value) for value in values):
            self._odom_stamp = time.monotonic()

    def _battery_callback(self, message: BatteryState) -> None:
        if math.isfinite(message.voltage):
            self._battery_voltage = float(message.voltage)
            self._battery_stamp = time.monotonic()

    def _publish_guarded_command(self) -> None:
        now = time.monotonic()
        decision = evaluate_guard(
            self._requested.linear.x,
            self._requested.angular.z,
            self._config,
            GuardState(
                now=now,
                command_stamp=self._command_stamp,
                scan_stamp=self._scan_stamp,
                odom_stamp=self._odom_stamp,
                battery_stamp=self._battery_stamp,
                battery_voltage=self._battery_voltage,
                scan_valid_points=self._scan_valid_points,
                front_min_range=self._front_min_range,
                front_hit_points=self._front_hit_points,
                rear_min_range=self._rear_min_range,
                rear_hit_points=self._rear_hit_points,
                left_min_range=self._left_min_range,
                right_min_range=self._right_min_range,
            ),
        )
        output = Twist()
        output.linear.x = decision.linear_x
        output.angular.z = decision.angular_z
        self._publisher.publish(output)
        if decision.reason != self._last_reason:
            # rclpy caches logging metadata per call site and refuses to see the
            # same site used at two severities ("Logger severity cannot be
            # changed between calls"), which killed this node -- and with it the
            # whole cmd_vel chain -- on the first permitted->blocked transition.
            # Two distinct call sites keep both severities legal.
            message = f"velocity guard state: {decision.reason}"
            if decision.permitted:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self._last_reason = decision.reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        for _ in range(3):
            node._publisher.publish(zero)  # final best-effort stop on orderly shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
