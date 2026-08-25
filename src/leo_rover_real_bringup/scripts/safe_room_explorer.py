#!/usr/bin/env python3

"""Run bounded room exploration with direction-aware collision recovery."""

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

from exploration_policy import (
    choose_escape_action,
    choose_turn_direction,
    turn_clearances,
)


class ExplorationComplete(Exception):
    """Leave the executor after the commanded stop interval."""


class SafeRoomExplorer(Node):
    """Publish cautious commands only through Nav2 Collision Monitor."""

    def __init__(self):
        super().__init__("safe_room_explorer")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/wheel_odom_integrated")
        self.declare_parameter("battery_topic", "/firmware/battery_averaged")
        self.declare_parameter("cmd_vel_request_topic", "/cmd_vel_request")
        self.declare_parameter("cmd_vel_output_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("angular_speed", 0.24)
        self.declare_parameter("run_duration", 180.0)
        self.declare_parameter("max_distance", 12.0)
        self.declare_parameter("planned_turn_distance", 1.5)
        self.declare_parameter("planned_turn_angle", 1.05)
        self.declare_parameter("minimum_battery_voltage", 5.0)
        self.declare_parameter("front_stop_distance", 0.90)
        self.declare_parameter("front_clear_distance", 1.10)
        self.declare_parameter("self_filter_radius", 0.05)
        # Sector maths below runs on raw scan angles.  When the LIDAR is not
        # mounted facing forward, this rotates the scan into the base frame.
        # Rover 4 mounts it backwards: base_footprint -> laser_frame yaw = pi.
        self.declare_parameter("scan_yaw_offset", 0.0)
        self.declare_parameter("minimum_turn_clearance", 0.45)
        self.declare_parameter("minimum_turn_progress", 0.15)
        self.declare_parameter("turn_timeout", 18.0)
        self.declare_parameter("sector_outlier_points", 5)
        self.declare_parameter("front_block_duration", 0.8)
        self.declare_parameter("reverse_speed", 0.04)
        self.declare_parameter("reverse_distance", 0.35)
        self.declare_parameter("reverse_timeout", 10.0)
        self.declare_parameter("minimum_reverse_clearance", 0.75)
        self.declare_parameter("maximum_reverse_attempts", 2)
        self.declare_parameter("scan_timeout", 0.4)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("battery_timeout", 1.0)
        self.declare_parameter("output_timeout", 1.0)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.battery_topic = self.get_parameter("battery_topic").value
        self.cmd_request_topic = self.get_parameter("cmd_vel_request_topic").value
        self.cmd_output_topic = self.get_parameter("cmd_vel_output_topic").value

        if self.cmd_request_topic.rstrip("/") in ("/cmd_vel", "/cmd_vel_raw"):
            raise RuntimeError("refusing to bypass the physical-rover safety chain")

        # Hard ceilings stay in force even if launch arguments are mistyped.
        self.linear_speed = min(
            abs(float(self.get_parameter("linear_speed").value)), 0.10
        )
        self.angular_speed = min(
            abs(float(self.get_parameter("angular_speed").value)), 0.30
        )
        self.run_duration = min(
            max(float(self.get_parameter("run_duration").value), 1.0), 180.0
        )
        self.max_distance = min(
            max(float(self.get_parameter("max_distance").value), 0.10), 12.0
        )
        self.planned_turn_distance = min(
            max(float(self.get_parameter("planned_turn_distance").value), 0.0),
            4.0,
        )
        self.planned_turn_angle = min(
            max(abs(float(self.get_parameter("planned_turn_angle").value)), 0.35),
            math.pi / 2.0,
        )
        self.minimum_battery = max(
            float(self.get_parameter("minimum_battery_voltage").value),
            5.0,
        )
        self.front_stop = float(
            self.get_parameter("front_stop_distance").value
        )
        self.front_clear = max(
            float(self.get_parameter("front_clear_distance").value),
            self.front_stop + 0.10,
        )
        self.self_filter_radius = min(
            max(float(self.get_parameter("self_filter_radius").value), 0.0),
            0.15,
        )
        self.scan_yaw_offset = self._normalize_angle(
            float(self.get_parameter("scan_yaw_offset").value)
        )
        self.minimum_turn_clearance = min(
            max(
                float(self.get_parameter("minimum_turn_clearance").value),
                0.35,
            ),
            0.75,
        )
        self.minimum_turn_progress = min(
            max(
                abs(float(self.get_parameter("minimum_turn_progress").value)),
                0.10,
            ),
            0.35,
        )
        self.turn_timeout = min(
            max(float(self.get_parameter("turn_timeout").value), 5.0),
            30.0,
        )
        self.sector_outlier_points = min(
            max(int(self.get_parameter("sector_outlier_points").value), 0),
            10,
        )
        self.front_block_duration = min(
            max(float(self.get_parameter("front_block_duration").value), 0.2),
            2.0,
        )
        self.reverse_speed = min(
            abs(float(self.get_parameter("reverse_speed").value)), 0.05
        )
        self.reverse_distance = min(
            max(float(self.get_parameter("reverse_distance").value), 0.10),
            0.50,
        )
        self.reverse_timeout = min(
            max(float(self.get_parameter("reverse_timeout").value), 3.0),
            15.0,
        )
        self.minimum_reverse_clearance = min(
            max(
                float(self.get_parameter("minimum_reverse_clearance").value),
                0.55,
            ),
            1.50,
        )
        self.maximum_reverse_attempts = min(
            max(int(self.get_parameter("maximum_reverse_attempts").value), 0),
            2,
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.battery_timeout = float(
            self.get_parameter("battery_timeout").value
        )
        self.output_timeout = float(self.get_parameter("output_timeout").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_request_topic, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            self.battery_topic,
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist, self.cmd_output_topic, self._output_callback, 10
        )

        now = time.monotonic()
        self.start_time = now
        self.scan_time = None
        self.odom_time = None
        self.battery_time = None
        self.output_time = None
        self.scan = None
        self._scan_angles = None
        self._scan_ranges = None
        self._scan_geometry = None
        self.battery = None
        self.last_output = Twist()
        self.first_position = None
        self.last_position = None
        self.yaw = None
        self.path_length = 0.0
        self.mode = "waiting"
        self.turn_direction = 1.0
        self.turn_started = None
        self.turn_start_yaw = None
        self.front_blocked_since = None
        self.reverse_started = None
        self.reverse_start_position = None
        self.reverse_attempts = 0
        self.forward_recovery_distance = 0.0
        self.next_planned_turn_distance = (
            self.planned_turn_distance
            if self.planned_turn_distance > 0.0
            else math.inf
        )
        self.motion_started = False
        self.motion_start_time = None
        self.output_held_since = None
        self.finish_time = None
        self.finish_reason = None
        self.last_log_time = 0.0
        self.timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            "bounded explorer ready: "
            f"linear={self.linear_speed:.2f} m/s, "
            f"angular={self.angular_speed:.2f} rad/s, "
            f"duration={self.run_duration:.1f} s, "
            f"distance_limit={self.max_distance:.2f} m, "
            f"planned_turn_every={self.planned_turn_distance:.2f} m, "
            f"reverse={self.reverse_speed:.2f} m/s for "
            f"{self.reverse_distance:.2f} m max"
        )

    def _scan_callback(self, msg):
        self.scan = msg
        # Six sector queries per control cycle used to re-walk every ray in
        # Python and call atan2 on each one.  That starved Collision Monitor
        # of CPU, which then dropped /scan as a stale source and stopped
        # publishing -- so cache the per-ray arrays once per scan instead.
        count = len(msg.ranges)
        key = (msg.angle_min, msg.angle_increment, count)
        if key != self._scan_geometry:
            angles = (
                float(msg.angle_min)
                + self.scan_yaw_offset
                + numpy.arange(count, dtype=float) * float(msg.angle_increment)
            )
            self._scan_angles = numpy.arctan2(
                numpy.sin(angles), numpy.cos(angles)
            )
            self._scan_geometry = key
        self._scan_ranges = numpy.asarray(msg.ranges, dtype=float)
        self.scan_time = time.monotonic()

    def _odom_callback(self, msg):
        now = time.monotonic()
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if self.first_position is None:
            self.first_position = position
        if self.last_position is not None:
            step = math.hypot(
                position[0] - self.last_position[0],
                position[1] - self.last_position[1],
            )
            if step < 0.20:
                self.path_length += step
                if self.mode == "forward" and self.reverse_attempts > 0:
                    self.forward_recovery_distance += step
                elif self.mode != "forward":
                    self.forward_recovery_distance = 0.0
        self.last_position = position
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_time = now

    def _battery_callback(self, msg):
        self.battery = float(msg.data)
        self.battery_time = time.monotonic()

    def _output_callback(self, msg):
        self.last_output = msg
        self.output_time = time.monotonic()

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _sector_clearance(self, lower_degrees, upper_degrees):
        if self.scan is None or self._scan_angles is None:
            return 0.0
        lower = math.radians(lower_degrees)
        upper = math.radians(upper_degrees)

        ranges = self._scan_ranges
        in_sector = (self._scan_angles >= lower) & (self._scan_angles <= upper)
        if not in_sector.any():
            return 0.0

        range_max = float(self.scan.range_max)
        minimum = max(float(self.scan.range_min), self.self_filter_radius)

        # A positive infinite reading means "nothing out to max range", which
        # is maximum clearance rather than a dropout.
        unbounded = in_sector & numpy.isposinf(ranges)
        bounded = (
            in_sector
            & numpy.isfinite(ranges)
            & (ranges >= minimum)
            & (ranges <= range_max)
        )

        values = numpy.concatenate((
            ranges[bounded],
            numpy.full(int(numpy.count_nonzero(unbounded)), range_max),
        ))
        if values.size < 5:
            return 0.0
        values.sort()
        return float(values[min(self.sector_outlier_points, values.size - 1)])

    def _publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _finish(self, reason):
        if self.finish_time is None:
            self.finish_time = time.monotonic()
            self.finish_reason = reason
            self.mode = "stopping"
            self.get_logger().info(
                f"stopping: {reason}; path_length={self.path_length:.3f} m"
            )
        self._publish()

    def _fresh(self, timestamp, timeout, now):
        return timestamp is not None and now - timestamp <= timeout

    def _readiness_problem(self, now):
        if (
            self.cmd_pub.get_subscription_count() < 1
            or self.count_publishers(self.cmd_output_topic) < 1
        ):
            return "command chain discovery incomplete"
        if not self._fresh(self.scan_time, self.scan_timeout, now):
            return "stale lidar scan"
        if not self._fresh(self.odom_time, self.odom_timeout, now):
            return "stale wheel odometry"
        if not self._fresh(self.battery_time, self.battery_timeout, now):
            return "stale battery telemetry"
        # Collision Monitor may stay silent while its input is zero.  Avoid a
        # startup deadlock by requiring its output only after the first
        # actionable request, with one output-timeout interval for handoff.
        if (
            self.motion_started
            and now - self.motion_start_time > max(2.0, self.output_timeout)
            and not self._fresh(self.output_time, self.output_timeout, now)
        ):
            return "collision monitor output is not live"
        if self.battery is None or self.battery < self.minimum_battery:
            voltage = "unknown" if self.battery is None else f"{self.battery:.2f} V"
            return f"battery below {self.minimum_battery:.2f} V ({voltage})"
        return None

    def _start_reverse(self, now, rear_clearance, reason):
        if self.reverse_speed <= 0.0:
            self._finish(f"{reason}; reverse disabled")
            return False
        if self.reverse_attempts >= self.maximum_reverse_attempts:
            self._finish(f"{reason}; reverse-attempt limit reached")
            return False
        if rear_clearance < self.minimum_reverse_clearance:
            self._finish(
                f"{reason}; rear clearance {rear_clearance:.2f} m is below "
                f"{self.minimum_reverse_clearance:.2f} m"
            )
            return False
        self.mode = "reversing"
        self.reverse_started = now
        self.reverse_start_position = self.last_position
        self.reverse_attempts += 1
        self.forward_recovery_distance = 0.0
        self.get_logger().info(
            f"reverse escape {self.reverse_attempts}/"
            f"{self.maximum_reverse_attempts}: {reason}; "
            f"rear={rear_clearance:.2f} m"
        )
        return True

    def _reverse_travel(self):
        if self.reverse_start_position is None or self.last_position is None:
            return 0.0
        return math.hypot(
            self.last_position[0] - self.reverse_start_position[0],
            self.last_position[1] - self.reverse_start_position[1],
        )

    def _begin_escape(
        self,
        now,
        left_turn_clearance,
        right_turn_clearance,
        rear_clearance,
        reason,
    ):
        action, direction = choose_escape_action(
            left_turn_clearance,
            right_turn_clearance,
            self.minimum_turn_clearance,
            rear_clearance,
            self.minimum_reverse_clearance,
            self.reverse_attempts,
            self.maximum_reverse_attempts,
        )
        if action == "turn":
            self.mode = "turning"
            self.turn_direction = direction
            self.turn_started = now
            self.turn_start_yaw = self.yaw
            return True
        if action == "reverse":
            return self._start_reverse(now, rear_clearance, reason)
        self._finish(f"{reason}; no safe turn or reverse corridor")
        return False

    def _timer_callback(self):
        now = time.monotonic()
        mode_at_start = self.mode

        if self.finish_time is not None:
            self._publish()
            if now - self.finish_time >= 2.0:
                self.timer.cancel()
                raise ExplorationComplete
            return

        problem = self._readiness_problem(now)
        if problem is not None:
            self._publish()
            if self.motion_started or now - self.start_time > 5.0:
                self._finish(problem)
            return

        if now - self.start_time >= self.run_duration:
            self._finish("wall-clock duration reached")
            return
        if self.path_length >= self.max_distance:
            self._finish("odometry distance limit reached")
            return

        front = self._sector_clearance(-32.0, 32.0)
        left = self._sector_clearance(25.0, 105.0)
        right = self._sector_clearance(-105.0, -25.0)
        rear_left = self._sector_clearance(100.0, 170.0)
        rear_right = self._sector_clearance(-170.0, -100.0)
        rear_center = min(
            self._sector_clearance(145.0, 179.9),
            self._sector_clearance(-179.9, -145.0),
        )
        if min(front, left, right, rear_left, rear_right, rear_center) <= 0.0:
            self._finish("lidar sector has insufficient valid samples")
            return

        # A turn sweeps the rear corner opposite the turn direction.  Choose
        # using both the forward-side and swept-rear clearances so a nominally
        # open right side cannot swing the rear-left corner into furniture.
        left_turn_clearance, right_turn_clearance = turn_clearances(
            left, right, rear_left, rear_right
        )

        if self.mode in ("waiting", "forward", "front_waiting"):
            if front < self.front_stop:
                if self.front_blocked_since is None:
                    self.front_blocked_since = now
                if now - self.front_blocked_since >= self.front_block_duration:
                    if not self._begin_escape(
                        now,
                        left_turn_clearance,
                        right_turn_clearance,
                        rear_center,
                        "persistent front obstacle",
                    ):
                        return
                else:
                    self.mode = "front_waiting"
            elif self.path_length >= self.next_planned_turn_distance:
                self.front_blocked_since = None
                self.mode = "planned_turning"
                self.turn_direction = choose_turn_direction(
                    left_turn_clearance, right_turn_clearance
                )
                self.turn_started = now
                self.turn_start_yaw = self.yaw
            else:
                self.front_blocked_since = None
                self.mode = "forward"
        elif self.mode == "turning":
            turn_clearance = (
                left_turn_clearance
                if self.turn_direction > 0.0
                else right_turn_clearance
            )
            turn_progress = (
                0.0
                if self.yaw is None or self.turn_start_yaw is None
                else abs(self._normalize_angle(self.yaw - self.turn_start_yaw))
            )
            if (
                now - self.turn_started >= 0.8
                and turn_progress >= self.minimum_turn_progress
                and front >= self.front_clear
                and turn_clearance >= self.minimum_turn_clearance
            ):
                self.mode = "forward"
                self.front_blocked_since = None
                if self.planned_turn_distance > 0.0:
                    self.next_planned_turn_distance = (
                        self.path_length + self.planned_turn_distance
                    )
            elif now - self.turn_started >= self.turn_timeout:
                if not self._start_reverse(
                    now, rear_center, "obstacle turn timed out"
                ):
                    return
        elif self.mode == "planned_turning":
            if self.yaw is None or self.turn_start_yaw is None:
                self._finish("turn heading unavailable")
                return
            turned = abs(
                self._normalize_angle(self.yaw - self.turn_start_yaw)
            )
            if turned >= self.planned_turn_angle and front >= self.front_stop:
                self.mode = "forward"
                self.front_blocked_since = None
                self.next_planned_turn_distance = (
                    self.path_length + self.planned_turn_distance
                )
            elif now - self.turn_started >= self.turn_timeout:
                if front >= self.front_stop:
                    self.mode = "forward"
                    self.front_blocked_since = None
                    self.next_planned_turn_distance = self.path_length + 0.75
                elif not self._start_reverse(
                    now, rear_center, "planned turn timed out near obstacle"
                ):
                    return
        elif self.mode == "reversing":
            reverse_travel = self._reverse_travel()
            if rear_center < self.minimum_reverse_clearance:
                self._finish(
                    f"reverse corridor closed at {rear_center:.2f} m"
                )
                return
            if (
                reverse_travel >= self.reverse_distance
                or now - self.reverse_started >= self.reverse_timeout
            ):
                self.mode = "turning"
                self.turn_direction = choose_turn_direction(
                    left_turn_clearance, right_turn_clearance
                )
                self.turn_started = now
                self.turn_start_yaw = self.yaw
                self.front_blocked_since = None

        if (
            self.mode == "forward"
            and self.reverse_attempts > 0
            and self.forward_recovery_distance >= 0.50
        ):
            self.get_logger().info(
                "reverse escape recovered; resetting consecutive-attempt limit"
            )
            self.reverse_attempts = 0
            self.forward_recovery_distance = 0.0

        if self.mode in ("turning", "planned_turning"):
            selected_clearance = (
                left_turn_clearance
                if self.turn_direction > 0.0
                else right_turn_clearance
            )
            alternate_clearance = (
                right_turn_clearance
                if self.turn_direction > 0.0
                else left_turn_clearance
            )
            if selected_clearance < self.minimum_turn_clearance:
                if alternate_clearance >= self.minimum_turn_clearance + 0.10:
                    self.turn_direction *= -1.0
                    self.turn_started = now
                    self.turn_start_yaw = self.yaw
                elif front >= self.front_stop:
                    self.mode = "forward"
                    self.front_blocked_since = None
                    self.next_planned_turn_distance = self.path_length + 0.75
                else:
                    if not self._start_reverse(
                        now,
                        rear_center,
                        "insufficient swept-corner turn clearance",
                    ):
                        return

        if self.mode != mode_at_start:
            self.output_held_since = None
        output_is_zero = (
            abs(self.last_output.linear.x) < 0.005
            and abs(self.last_output.angular.z) < 0.01
        )
        active_motion_mode = self.mode in (
            "forward",
            "turning",
            "planned_turning",
            "reversing",
        )
        if output_is_zero and active_motion_mode:
            if self.output_held_since is None:
                self.output_held_since = now
            elif now - self.output_held_since > 1.2:
                blocked_mode = self.mode
                self.output_held_since = None
                if blocked_mode == "forward":
                    if not self._begin_escape(
                        now,
                        left_turn_clearance,
                        right_turn_clearance,
                        rear_center,
                        "collision monitor blocked forward motion",
                    ):
                        return
                elif blocked_mode in ("turning", "planned_turning"):
                    if not self._start_reverse(
                        now,
                        rear_center,
                        "collision monitor blocked turning motion",
                    ):
                        return
                else:
                    self._finish("collision monitor blocked reverse motion")
                    return
        else:
            self.output_held_since = None

        if self.mode == "forward":
            command_linear = self.linear_speed
            command_angular = 0.0
        elif self.mode == "front_waiting":
            command_linear = 0.0
            command_angular = 0.0
        elif self.mode == "reversing":
            command_linear = -self.reverse_speed
            command_angular = 0.0
        else:
            command_linear = 0.0
            command_angular = self.turn_direction * self.angular_speed

        self._publish(command_linear, command_angular)
        if not self.motion_started:
            self.motion_started = True
            self.motion_start_time = now

        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f"mode={self.mode} front={front:.2f} left={left:.2f} "
                f"right={right:.2f} rear_left={rear_left:.2f} "
                f"rear_right={rear_right:.2f} rear={rear_center:.2f} "
                f"battery={self.battery:.2f} V "
                f"path={self.path_length:.2f} m"
            )

    def publish_final_zeros(self):
        for _ in range(20):
            self._publish()
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafeRoomExplorer()
        rclpy.spin(node)
    except (ExplorationComplete, KeyboardInterrupt, ExternalShutdownException):
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
