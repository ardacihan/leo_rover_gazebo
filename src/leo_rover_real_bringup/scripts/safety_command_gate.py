#!/usr/bin/env python3

"""Fail-closed velocity gate for the physical Leo Rover."""

import copy
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from exploration_policy import robust_clearance, scan_yaw_from_transform


class SafetyCommandGate(Node):
    """Forward bounded commands only while every safety input is fresh."""

    def __init__(self):
        super().__init__("safety_command_gate")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("filtered_scan_topic", "/scan_self_filtered")
        self.declare_parameter("publish_filtered_scan", True)
        self.declare_parameter("camera_scan_topic", "/camera/scan")
        self.declare_parameter("require_camera_scan", True)
        self.declare_parameter("camera_scan_timeout", 0.5)
        # Any return closer than the footprint half-width is the rover itself
        # (mast, brackets, cabling).  On Rover 4 every persistent close return
        # measured 0.023-0.174 m, all of it structure, so a radial mask removes
        # the whole self-signature without a fragile per-angle window.
        self.declare_parameter("self_mask_radius", 0.22)
        # Scan angles are expressed in the lidar frame.  Rover 4 mounts the
        # lidar yawed by pi, so raw scan angles are rotated 180 degrees from
        # base_footprint.  Left NaN this is resolved from TF at runtime.
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("scan_yaw_offset", float("nan"))
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("battery_topic", "/firmware/battery_averaged")
        self.declare_parameter("cmd_vel_request_topic", "/cmd_vel_request")
        self.declare_parameter("cmd_vel_raw_topic", "/cmd_vel_raw")
        self.declare_parameter("cmd_vel_output_topic", "/cmd_vel")
        self.declare_parameter(
            "allowed_cmd_vel_output_publishers", ["collision_monitor"]
        )
        # Nodes permitted to subscribe to the gate output. Collision Monitor is
        # the consumer; the incident recorder only observes, so that it can tell
        # a gate refusal apart from a Collision Monitor veto when capturing.
        self.declare_parameter(
            "allowed_cmd_vel_raw_subscribers",
            ["collision_monitor", "incident_recorder"],
        )
        self.declare_parameter(
            "conditionally_disabled_output_publisher", "robot_supervisor_rgb"
        )
        self.declare_parameter("conditional_publisher_parameter", "enabled")
        self.declare_parameter("conditional_publisher_state_timeout", 0.75)
        self.declare_parameter("maximum_linear_speed", 0.10)
        self.declare_parameter("maximum_reverse_speed", 0.0)
        self.declare_parameter("maximum_angular_speed", 0.30)
        self.declare_parameter("minimum_battery_voltage", 10.20)
        self.declare_parameter("minimum_finite_scan_points", 30)
        self.declare_parameter("minimum_reverse_clearance", 0.75)
        self.declare_parameter("self_filter_radius", 0.05)
        self.declare_parameter("rear_outlier_points", 5)
        self.declare_parameter("scan_timeout", 0.4)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("battery_timeout", 1.0)
        self.declare_parameter("command_timeout", 0.3)

        scan_topic = self.get_parameter("scan_topic").value
        filtered_scan_topic = self.get_parameter("filtered_scan_topic").value
        self.publish_filtered_scan = bool(
            self.get_parameter("publish_filtered_scan").value
        )
        camera_scan_topic = self.get_parameter("camera_scan_topic").value
        self.require_camera_scan = bool(self.get_parameter("require_camera_scan").value)
        self.camera_scan_timeout = float(self.get_parameter("camera_scan_timeout").value)
        self.self_mask_radius = min(
            max(float(self.get_parameter("self_mask_radius").value), 0.0), 0.25
        )
        self.base_frame = self.get_parameter("base_frame").value
        self.scan_yaw_offset = float(
            self.get_parameter("scan_yaw_offset").value
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        odom_topic = self.get_parameter("odom_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        request_topic = self.get_parameter("cmd_vel_request_topic").value
        raw_topic = self.get_parameter("cmd_vel_raw_topic").value
        self.cmd_vel_output_topic = self.get_parameter("cmd_vel_output_topic").value
        self.allowed_output_publishers = {
            str(name).lstrip("/")
            for name in self.get_parameter(
                "allowed_cmd_vel_output_publishers"
            ).value
        }
        self.conditional_output_publisher = str(
            self.get_parameter("conditionally_disabled_output_publisher").value
        ).lstrip("/")
        self.conditional_publisher_parameter = str(
            self.get_parameter("conditional_publisher_parameter").value
        )
        self.conditional_publisher_state_timeout = max(
            float(
                self.get_parameter("conditional_publisher_state_timeout").value
            ),
            0.2,
        )
        self.conditional_publisher_client = self.create_client(
            GetParameters,
            f"/{self.conditional_output_publisher}/get_parameters",
        )
        self.conditional_publisher_future = None
        self.conditional_publisher_enabled = None
        self.conditional_publisher_state_time = None
        self.conditional_publisher_request_time = None
        self.request_topic = request_topic
        self.raw_topic = raw_topic
        self.allowed_raw_subscribers = {
            str(name).lstrip("/")
            for name in self.get_parameter(
                "allowed_cmd_vel_raw_subscribers"
            ).value
        }

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
        # Operator-owned floor.  The firmware's own cutoff is 10.0 V, so a
        # value at or below that disables the gate's battery check entirely.
        self.minimum_battery = float(
            self.get_parameter("minimum_battery_voltage").value
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
        self.filtered_scan_publisher = None
        if self.publish_filtered_scan:
            filtered_scan_qos = QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE
            )
            self.filtered_scan_publisher = self.create_publisher(
                LaserScan, filtered_scan_topic, filtered_scan_qos
            )
        self.create_subscription(
            Twist, request_topic, self._command_callback, 10
        )
        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, camera_scan_topic, self._camera_scan_callback,
            qos_profile_sensor_data
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
        self.camera_scan_time = None
        self.odom_time = None
        self.battery_time = None
        self.battery = None
        self.finite_scan_points = 0
        self.scan_yaw_resolved = False
        self.rear_clearance = None
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

    def _resolve_scan_yaw(self, frame_id):
        """Return the lidar-to-base yaw, resolving it from TF once if needed."""
        if not math.isnan(self.scan_yaw_offset):
            return self.scan_yaw_offset
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        self.scan_yaw_offset = scan_yaw_from_transform(transform)
        self.get_logger().info(
            f"resolved {self.base_frame} <- {frame_id} yaw offset "
            f"{math.degrees(self.scan_yaw_offset):.1f} deg from TF"
        )
        return self.scan_yaw_offset

    def _scan_callback(self, msg):
        finite_points = 0
        rear_distances = []
        if self.filtered_scan_publisher is not None:
            filtered = copy.deepcopy(msg)
            filtered_ranges = [
                float("inf")
                if math.isfinite(value) and value <= self.self_mask_radius
                else value
                for value in filtered.ranges
            ]
            filtered.ranges = filtered_ranges
            self.filtered_scan_publisher.publish(filtered)
            msg = filtered

        # The rear corridor must be measured in the base frame, not in raw
        # scan angles: a yawed lidar mount would otherwise report the forward
        # corridor and authorise a blind reverse.
        yaw_offset = self._resolve_scan_yaw(msg.header.frame_id)
        angle = float(msg.angle_min)
        for value in msg.ranges:
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                finite_points += 1
                if value >= self.self_filter_radius and yaw_offset is not None:
                    base_angle = angle + yaw_offset
                    x = float(value) * math.cos(base_angle)
                    y = float(value) * math.sin(base_angle)
                    if x < 0.0 and abs(y) <= 0.30:
                        rear_distances.append(-x)
            angle += float(msg.angle_increment)
        self.scan_yaw_resolved = yaw_offset is not None
        self.finite_scan_points = finite_points
        self.rear_clearance = robust_clearance(
            rear_distances,
            self.rear_outlier_points,
            default_clearance=float(msg.range_max),
        )
        self.scan_time = time.monotonic()

    def _camera_scan_callback(self, _msg):
        self.camera_scan_time = time.monotonic()

    def _odom_callback(self, _msg):
        self.odom_messages += 1
        self.odom_time = time.monotonic()

    def _battery_callback(self, msg):
        self.battery = float(msg.data)
        self.battery_time = time.monotonic()

    @staticmethod
    def _fresh(timestamp, timeout, now):
        return timestamp is not None and now - timestamp <= timeout

    def _poll_conditional_publisher(self, now):
        future = self.conditional_publisher_future
        if future is not None and future.done():
            self.conditional_publisher_future = None
            try:
                response = future.result()
                value = response.values[0]
                if value.type != ParameterType.PARAMETER_BOOL:
                    raise ValueError("parameter is not boolean")
                self.conditional_publisher_enabled = bool(value.bool_value)
                self.conditional_publisher_state_time = now
            except (IndexError, RuntimeError, ValueError) as error:
                self.conditional_publisher_enabled = None
                self.conditional_publisher_state_time = None
                self.get_logger().warning(
                    f"could not verify {self.conditional_output_publisher} is disabled: {error}"
                )

        request_age = (
            math.inf
            if self.conditional_publisher_request_time is None
            else now - self.conditional_publisher_request_time
        )
        if (
            self.conditional_publisher_future is None
            and request_age >= 0.25
            and self.conditional_publisher_client.service_is_ready()
        ):
            request = GetParameters.Request()
            request.names = [self.conditional_publisher_parameter]
            self.conditional_publisher_future = (
                self.conditional_publisher_client.call_async(request)
            )
            self.conditional_publisher_request_time = now

    def _blocking_reason(self, now):
        if self.count_publishers(self.request_topic) > 1:
            return "multiple command-request publishers"
        # Check *which* node consumes the gate output, not merely how many do.
        # A bare count of one passes even when the single subscriber is a rogue
        # consumer and Collision Monitor is absent, and it fails on a passive
        # observer such as the incident recorder, which never publishes.
        raw_names = {
            info.node_name.lstrip("/")
            for info in self.get_subscriptions_info_by_topic(self.raw_topic)
        }
        if "collision_monitor" not in raw_names:
            return "Collision Monitor is not subscribed to the gate output"
        unexpected = raw_names - self.allowed_raw_subscribers
        if unexpected:
            return "unexpected raw command consumers: " + ", ".join(
                sorted(unexpected)
            )
        output_infos = self.get_publishers_info_by_topic(
            self.cmd_vel_output_topic
        )
        output_names = {info.node_name.lstrip("/") for info in output_infos}
        unexpected = output_names - self.allowed_output_publishers
        if unexpected:
            return "unexpected final command publishers: " + ", ".join(
                sorted(unexpected)
            )
        if "collision_monitor" not in output_names:
            return "Collision Monitor final publisher is absent"
        if self.conditional_output_publisher in output_names:
            self._poll_conditional_publisher(now)
            if not self._fresh(
                self.conditional_publisher_state_time,
                self.conditional_publisher_state_timeout,
                now,
            ):
                return (
                    f"{self.conditional_output_publisher} disabled state is stale"
                )
            if self.conditional_publisher_enabled is not False:
                return f"{self.conditional_output_publisher} is enabled"
        if not self._fresh(self.command_time, self.command_timeout, now):
            return "stale command"
        if not self._fresh(self.scan_time, self.scan_timeout, now):
            return "stale lidar scan"
        if self.require_camera_scan and not self._fresh(
            self.camera_scan_time, self.camera_scan_timeout, now
        ):
            return "stale depth-camera scan"
        if self.finite_scan_points < self.minimum_finite_points:
            return f"only {self.finite_scan_points} finite scan points"
        if not self.scan_yaw_resolved:
            return f"lidar-to-{self.base_frame} transform unavailable"
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
