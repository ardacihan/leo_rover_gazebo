#!/usr/bin/env python3

"""Run bounded room exploration with direction-aware collision recovery."""

import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from exploration_policy import (
    choose_escape_action,
    choose_turn_direction,
    scan_yaw_from_transform,
    turn_clearances,
)


class ExplorationComplete(Exception):
    """Leave the executor after the commanded stop interval."""


class SafeRoomExplorer(Node):
    """Publish cautious commands only through Nav2 Collision Monitor."""

    def __init__(self):
        super().__init__("safe_room_explorer")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("camera_scan_topic", "/camera/scan")
        self.declare_parameter("camera_scan_timeout", 0.5)
        self.declare_parameter("sensor_recovery_limit", 45.0)
        # Narrow-passage (doorway) traversal: when the fused scan shows a
        # robot-wide free gap ahead, steer into it at reduced speed with a
        # tighter stop threshold. Collision Monitor's true-footprint check
        # remains the hard guard throughout.
        self.declare_parameter("passage_enable", True)
        self.declare_parameter("passage_speed", 0.08)
        self.declare_parameter("passage_front_stop", 0.35)
        self.declare_parameter("gap_range", 1.4)
        self.declare_parameter("gap_min_width", 0.55)
        self.declare_parameter("gap_max_edge_range", 2.0)
        self.declare_parameter("gap_steer_gain", 0.6)
        self.declare_parameter("gap_steer_cap", 0.12)
        # Sectors are named in base-frame terms (front, left, rear).  Each scan
        # source carries its own mounting yaw, resolved from TF when left NaN.
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("scan_yaw_offset", float("nan"))
        self.declare_parameter("camera_scan_yaw_offset", float("nan"))
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
        self.declare_parameter("minimum_battery_voltage", 10.20)
        self.declare_parameter("front_stop_distance", 0.55)
        self.declare_parameter("front_clear_distance", 0.70)
        self.declare_parameter("self_filter_radius", 0.05)
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
        self.camera_scan_topic = self.get_parameter("camera_scan_topic").value
        self.camera_scan_timeout = float(self.get_parameter("camera_scan_timeout").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.scan_yaw_offset = float(self.get_parameter("scan_yaw_offset").value)
        self.camera_scan_yaw_offset = float(
            self.get_parameter("camera_scan_yaw_offset").value
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.battery_topic = self.get_parameter("battery_topic").value
        self.cmd_request_topic = self.get_parameter("cmd_vel_request_topic").value
        self.cmd_output_topic = self.get_parameter("cmd_vel_output_topic").value

        if self.cmd_request_topic.rstrip("/") in ("/cmd_vel", "/cmd_vel_raw"):
            raise RuntimeError("refusing to bypass the physical-rover safety chain")

        # Hard ceilings stay in force even if launch arguments are mistyped.
        # Raised 2026-08-14 on operator request after validated runs: speed
        # 0.10 -> 0.15 (gate clamps to 0.15 too), duration 180 -> 600 s,
        # distance 12 -> 1000 m (coverage runs are ended by the coverage
        # watcher or the wall clock, not by path length).
        self.linear_speed = min(
            abs(float(self.get_parameter("linear_speed").value)), 0.18
        )
        self.angular_speed = min(
            abs(float(self.get_parameter("angular_speed").value)), 0.30
        )
        self.run_duration = min(
            max(float(self.get_parameter("run_duration").value), 1.0), 600.0
        )
        self.max_distance = min(
            max(float(self.get_parameter("max_distance").value), 0.10), 1000.0
        )
        self.planned_turn_distance = min(
            max(float(self.get_parameter("planned_turn_distance").value), 0.0),
            4.0,
        )
        self.planned_turn_angle = min(
            max(abs(float(self.get_parameter("planned_turn_angle").value)), 0.35),
            math.pi / 2.0,
        )
        # Operator-owned floor; see safety_command_gate.py.
        self.minimum_battery = float(
            self.get_parameter("minimum_battery_voltage").value
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
        # Floor lowered 0.35 -> 0.32 (2026-08-14): the in-place corner sweep
        # is 0.311 m, and a 0.34 m pocket blocked every escape and ended a
        # run that a legal turn would have continued.
        self.minimum_turn_clearance = min(
            max(
                float(self.get_parameter("minimum_turn_clearance").value),
                0.32,
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
        self.sensor_recovery_limit = float(
            self.get_parameter("sensor_recovery_limit").value
        )
        self.passage_enable = bool(self.get_parameter("passage_enable").value)
        self.passage_speed = min(
            abs(float(self.get_parameter("passage_speed").value)), 0.10
        )
        self.passage_front_stop = max(
            float(self.get_parameter("passage_front_stop").value), 0.30
        )
        self.gap_range = float(self.get_parameter("gap_range").value)
        self.gap_min_width = max(
            float(self.get_parameter("gap_min_width").value), 0.52
        )
        self.gap_max_edge_range = float(
            self.get_parameter("gap_max_edge_range").value
        )
        self.gap_steer_gain = float(self.get_parameter("gap_steer_gain").value)
        self.gap_steer_cap = min(
            abs(float(self.get_parameter("gap_steer_cap").value)), 0.25
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_request_topic, 10)
        # Tells footprint_publisher to switch to the slim passage profile.
        self.passage_pub = self.create_publisher(Bool, "/passage_active", 5)
        # Handles are kept so a starved subscription can be destroyed and
        # rebuilt: a per-endpoint DDS failure can stop delivery to one reader
        # while the topic still flows to every other consumer (observed three
        # times on 2026-08-14; the bag and the gate received every message
        # while this node's reader went silent).
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.sub_camera = self.create_subscription(
            LaserScan, self.camera_scan_topic, self._camera_scan_callback,
            qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.sub_battery = self.create_subscription(
            Float32,
            self.battery_topic,
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.sub_output = self.create_subscription(
            Twist, self.cmd_output_topic, self._output_callback, 10
        )

        now = time.monotonic()
        self.start_time = now
        self.scan_time = None
        self.camera_scan_time = None
        self.odom_time = None
        self.battery_time = None
        self.output_time = None
        self.scan = None
        self.scan_yaw = None
        self.camera_scan = None
        self.camera_scan_yaw = None
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
        self.output_active_since = None
        self.finish_time = None
        self.finish_reason = None
        self.last_log_time = 0.0
        self.recovering_since = None
        self.recovering_problem = None
        self.last_resubscribe = 0.0
        self.stall_total = 0.0
        self.boxed_started = None
        self.probe_index = 0
        self.probe_started = None
        self.probe_start_yaw = None
        self.probe_start_position = None
        self.probe_command = (0.0, 0.0)
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
        self.scan_yaw = self._resolve_yaw(
            msg.header.frame_id, self.scan_yaw_offset
        )
        self.scan_time = time.monotonic()

    def _camera_scan_callback(self, msg):
        self.camera_scan = msg
        self.camera_scan_yaw = self._resolve_yaw(
            msg.header.frame_id, self.camera_scan_yaw_offset
        )
        self.camera_scan_time = time.monotonic()

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

    def _resolve_yaw(self, frame_id, override):
        """Return a scan source's mounting yaw, from the override or from TF."""
        if not math.isnan(override):
            return override
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        return scan_yaw_from_transform(transform)

    def _sector_clearance(self, lower_degrees, upper_degrees, scan=None,
                          yaw=None):
        if scan is None:
            scan, yaw = self.scan, self.scan_yaw
        if scan is None or yaw is None:
            return 0.0
        lower = math.radians(lower_degrees)
        upper = math.radians(upper_degrees)
        values = []
        angle = float(scan.angle_min)
        for reading in scan.ranges:
            normalized = self._normalize_angle(angle + yaw)
            if lower <= normalized <= upper:
                if math.isinf(reading) and reading > 0.0:
                    values.append(float(scan.range_max))
                elif (
                    math.isfinite(reading)
                    and reading >= max(
                        scan.range_min, self.self_filter_radius
                    )
                    and reading <= scan.range_max
                ):
                    values.append(float(reading))
            angle += float(scan.angle_increment)
        if len(values) < 5:
            return 0.0
        values.sort()
        return values[min(self.sector_outlier_points, len(values) - 1)]

    def _best_front_gap(self):
        """Find the widest free gap ahead in the fused scan (base frame).

        Returns (center_angle, passable_width, edge_range) or None. A ray is
        free when it reads beyond gap_range or has no return. The passable
        width is the chord between the two blocked edge points bounding the
        window — physically, the doorway posts.
        """
        scan = self.scan
        if scan is None or self.scan_yaw is None:
            return None
        half_fov = 0.84  # +/- 48 degrees in the base frame
        rays = []  # (base_angle, range, is_free)
        angle = float(scan.angle_min)
        for reading in scan.ranges:
            base = self._normalize_angle(angle + self.scan_yaw)
            angle += float(scan.angle_increment)
            if abs(base) > half_fov:
                continue
            r = float(reading)
            valid = math.isfinite(r) and r >= max(
                scan.range_min, self.self_filter_radius
            )
            free = (not valid) or r >= self.gap_range
            rays.append((base, r if valid else math.inf, free))
        if len(rays) < 8:
            return None
        rays.sort(key=lambda x: x[0])
        best = None
        i = 0
        n = len(rays)
        while i < n:
            if not rays[i][2]:
                i += 1
                continue
            j = i
            while j + 1 < n and rays[j + 1][2]:
                j += 1
            # blocked neighbours bound the gap; skip windows at the FOV edge
            if i > 0 and j < n - 1:
                a1, r1, _ = rays[i - 1]
                a2, r2, _ = rays[j + 1]
                r1 = min(r1, self.gap_range)
                r2 = min(r2, self.gap_range)
                width = math.sqrt(
                    max(
                        r1 * r1 + r2 * r2
                        - 2.0 * r1 * r2 * math.cos(a2 - a1),
                        0.0,
                    )
                )
                # Aim at the CARTESIAN midpoint of the two edge posts, not
                # the angular midpoint: with one post near and one far the
                # angular centre still drives into the near post (pinned the
                # robot at a door mouth for 120 s on 2026-08-14).
                x_mid = 0.5 * (r1 * math.cos(a1) + r2 * math.cos(a2))
                y_mid = 0.5 * (r1 * math.sin(a1) + r2 * math.sin(a2))
                center = math.atan2(y_mid, x_mid)
                edge = min(r1, r2)
                # Doorway-band windows outrank open space, else a wide free
                # area beside the door would mask it.
                in_band = self.gap_min_width <= width <= 1.30
                score = width + (10.0 if in_band else 0.0)
                if best is None or score > best[3]:
                    best = (center, width, edge, score)
            i = j + 1
        return best[:3] if best is not None else None

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
        if not self._fresh(
            self.camera_scan_time, self.camera_scan_timeout, now
        ):
            return "stale depth-camera scan"
        # Without a mounting yaw every sector would be mislabelled, so treat an
        # unresolved transform as a hard stop rather than assuming alignment.
        if self.scan_yaw is None:
            return f"lidar-to-{self.base_frame} transform unavailable"
        if self.camera_scan_yaw is None:
            return f"camera-to-{self.base_frame} transform unavailable"
        if not self._fresh(self.odom_time, self.odom_timeout, now):
            return "stale wheel odometry"
        if not self._fresh(self.battery_time, self.battery_timeout, now):
            return "stale battery telemetry"
        # Collision Monitor silence is handled by the hold detector (silence
        # while requesting motion == held at zero -> escape). Treating it as
        # a recoverable data stall here fought the hold detector: the pause
        # reset the hold timer every 2 s and the escape never fired, pinning
        # the robot at a door post on 2026-08-14.
        if self.battery is None or self.battery < self.minimum_battery:
            voltage = "unknown" if self.battery is None else f"{self.battery:.2f} V"
            return f"battery below {self.minimum_battery:.2f} V ({voltage})"
        return None

    # Data-flow stalls that a rebuilt subscription can cure. Structural
    # problems (missing transforms, low battery, absent command chain) are
    # deliberately NOT here and remain fatal.
    RECOVERABLE_PROBLEMS = {
        "stale lidar scan": "sub_scan",
        "stale depth-camera scan": "sub_camera",
        "stale wheel odometry": "sub_odom",
        "stale battery telemetry": "sub_battery",
    }

    def _resubscribe(self, attr):
        spec = {
            "sub_scan": (
                LaserScan, self.scan_topic, self._scan_callback,
                qos_profile_sensor_data,
            ),
            "sub_camera": (
                LaserScan, self.camera_scan_topic, self._camera_scan_callback,
                qos_profile_sensor_data,
            ),
            "sub_odom": (
                Odometry, self.odom_topic, self._odom_callback,
                qos_profile_sensor_data,
            ),
            "sub_battery": (
                Float32, self.battery_topic, self._battery_callback,
                qos_profile_sensor_data,
            ),
            "sub_output": (
                Twist, self.cmd_output_topic, self._output_callback, 10,
            ),
        }[attr]
        try:
            self.destroy_subscription(getattr(self, attr))
        except Exception:  # noqa: BLE001 - a dead handle must not stop recovery
            pass
        setattr(self, attr, self.create_subscription(*spec))

    def _enter_boxed(self, now, reason):
        """Probe low-speed escapes forever instead of ending the mission.

        Collision Monitor simulates the true chassis polygon against the
        fused scan and vetoes any colliding command, so cycling slow probe
        motions through it is safe even when the explorer's coarse sector
        floors see no legal escape. Added 2026-08-14 after two runs ended
        with 'no safe turn or reverse corridor' in survivable pockets.
        """
        if self.mode != "boxed_probe":
            self.get_logger().warning(
                f"boxed in ({reason}); probing low-speed escapes under "
                "Collision Monitor guard instead of stopping"
            )
            self.boxed_started = now
            self.probe_index = 0
            self.probe_started = now
            self.probe_start_yaw = self.yaw
            self.probe_start_position = self.last_position
            self.probe_command = (0.0, 0.0)
            self.mode = "boxed_probe"

    def _start_reverse(self, now, rear_clearance, reason):
        if self.reverse_speed <= 0.0:
            self.get_logger().warning(f"{reason}; reverse disabled")
            return False
        if self.reverse_attempts >= self.maximum_reverse_attempts:
            self.get_logger().warning(
                f"{reason}; reverse-attempt limit reached"
            )
            return False
        if rear_clearance < self.minimum_reverse_clearance:
            self.get_logger().warning(
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
            if self._start_reverse(now, rear_clearance, reason):
                return True
            self._enter_boxed(now, reason)
            return True
        self._enter_boxed(now, f"{reason}; no safe turn or reverse corridor")
        return True

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
            recover_attr = self.RECOVERABLE_PROBLEMS.get(problem)
            if recover_attr is None or (
                not self.motion_started and now - self.start_time <= 15.0
            ):
                if self.motion_started or now - self.start_time > 15.0:
                    self._finish(problem)
                return
            # Hold position and rebuild the starved reader instead of
            # aborting the mission; only an unrecovered stall is fatal.
            if self.recovering_since is None:
                self.recovering_since = now
                self.recovering_problem = problem
                self.last_resubscribe = 0.0
                # While paused only zeros are requested, so Collision Monitor
                # goes legitimately silent. Re-arm the output-liveness
                # requirement the same way startup does, or the pause would
                # cascade into a bogus "output is not live" recovery.
                self.motion_started = False
                self.motion_start_time = None
                self.get_logger().warning(
                    f"pausing: {problem}; rebuilding subscription and waiting "
                    f"up to {self.sensor_recovery_limit:.0f} s"
                )
            if now - self.last_resubscribe >= 3.0:
                self.last_resubscribe = now
                self._resubscribe(recover_attr)
            if now - self.recovering_since > self.sensor_recovery_limit:
                self._finish(
                    f"{self.recovering_problem}; not recovered within "
                    f"{self.sensor_recovery_limit:.0f} s"
                )
            return
        if self.recovering_since is not None:
            stalled = now - self.recovering_since
            self.stall_total += stalled
            self.get_logger().warning(
                f"recovered from '{self.recovering_problem}' after "
                f"{stalled:.1f} s; resuming exploration"
            )
            self.recovering_since = None
            self.recovering_problem = None
            self.front_blocked_since = None
            self.output_held_since = None
            self.output_active_since = None
            # Refresh maneuver clocks so a pause longer than turn/reverse
            # timeouts is not mistaken for a stuck maneuver on resume.
            if self.turn_started is not None:
                self.turn_started = now
            if self.reverse_started is not None:
                self.reverse_started = now

        # Stall time is excluded so a recovered pause does not eat the
        # exploration budget.
        if now - self.start_time - self.stall_total >= self.run_duration:
            self._finish("wall-clock duration reached")
            return
        if self.path_length >= self.max_distance:
            self._finish("odometry distance limit reached")
            return

        front = self._sector_clearance(-32.0, 32.0)
        camera_front = self._sector_clearance(
            -32.0, 32.0, self.camera_scan, self.camera_scan_yaw
        )
        front = min(front, camera_front)
        left = self._sector_clearance(25.0, 105.0)
        right = self._sector_clearance(-105.0, -25.0)
        rear_left = self._sector_clearance(100.0, 170.0)
        rear_right = self._sector_clearance(-170.0, -100.0)
        rear_center = min(
            self._sector_clearance(145.0, 179.9),
            self._sector_clearance(-179.9, -145.0),
        )
        # A sector with under five valid returns reads 0.0. That happens in
        # normal operation when a sector faces geometry beyond the C1's ~12 m
        # effective range (long corridor, open doorway), so it must NOT be
        # fatal: 0.0 already means "treat as blocked" everywhere below, which
        # steers the robot toward directions it can actually measure.
        if min(front, left, right, rear_left, rear_right, rear_center) <= 0.0:
            if now - self.last_log_time >= 2.0:
                self.last_log_time = now
                self.get_logger().warning(
                    "sector(s) with insufficient lidar samples treated as "
                    f"blocked: front={front:.2f} left={left:.2f} "
                    f"right={right:.2f} rear_left={rear_left:.2f} "
                    f"rear_right={rear_right:.2f} rear={rear_center:.2f}"
                )

        # A turn sweeps the rear corner opposite the turn direction.  Choose
        # using both the forward-side and swept-rear clearances so a nominally
        # open right side cannot swing the rear-left corner into furniture.
        left_turn_clearance, right_turn_clearance = turn_clearances(
            left, right, rear_left, rear_right
        )

        # Doorway handling: a qualifying free gap ahead switches to passage
        # mode — steer toward the gap centre, slow down, and tolerate a
        # closer front reading (the door frame enters the front wedge before
        # the robot is through). Collision Monitor still vetoes real contact.
        passage_gap = None
        if (
            self.passage_enable
            and self.mode in ("waiting", "forward", "front_waiting")
        ):
            gap = self._best_front_gap()
            # Only a NARROW free window is a doorway; a wide one is open
            # space and must not slow the robot down.
            if (
                gap is not None
                and self.gap_min_width <= gap[1] <= 1.30
                and gap[2] <= self.gap_max_edge_range
            ):
                passage_gap = gap
                if now - self.last_log_time >= 2.0:
                    self.get_logger().info(
                        f"passage mode: gap width={gap[1]:.2f} m at "
                        f"{math.degrees(gap[0]):.0f} deg, edge={gap[2]:.2f} m"
                    )
        self.passage_pub.publish(Bool(data=passage_gap is not None))
        front_stop_eff = (
            self.passage_front_stop if passage_gap else self.front_stop
        )

        if self.mode in ("waiting", "forward", "front_waiting"):
            if front < front_stop_eff:
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
            elif passage_gap is None and self.path_length >= self.next_planned_turn_distance:
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
                    self._enter_boxed(now, "obstacle turn timed out")
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
                    self._enter_boxed(
                        now, "planned turn timed out near obstacle"
                    )
        elif self.mode == "reversing":
            reverse_travel = self._reverse_travel()
            if rear_center < self.minimum_reverse_clearance:
                # Abort the maneuver, not the mission: stop reversing at once
                # and re-evaluate with a turn. A sparse rear sector reads 0.0
                # and must not end the run; genuinely boxed robots still stop
                # through the reverse-attempt limit and escape logic.
                self.get_logger().warning(
                    f"reverse corridor closed at {rear_center:.2f} m; "
                    "switching to turning"
                )
                self.mode = "turning"
                self.turn_direction = choose_turn_direction(
                    left_turn_clearance, right_turn_clearance
                )
                self.turn_started = now
                self.turn_start_yaw = self.yaw
                self.front_blocked_since = None
                self._publish()
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
        elif self.mode == "boxed_probe":
            wider = 1.0 if left_turn_clearance >= right_turn_clearance else -1.0
            candidates = [(0.0, wider), (0.0, -wider)]
            if self.reverse_speed > 0.0:
                candidates.append((-1.0, 0.0))
            candidates.append((1.0, 0.0))
            progressed = False
            if (
                self.probe_start_yaw is not None
                and self.yaw is not None
                and abs(
                    self._normalize_angle(self.yaw - self.probe_start_yaw)
                ) > 0.12
            ):
                progressed = True
            if (
                self.probe_start_position is not None
                and self.last_position is not None
                and math.hypot(
                    self.last_position[0] - self.probe_start_position[0],
                    self.last_position[1] - self.probe_start_position[1],
                ) > 0.04
            ):
                progressed = True
            # Exit without progress only when the SIDES opened too — the
            # front wedge can read clear while a post still pins the flank
            # (spurious instant exits observed at a door on 2026-08-14).
            if (
                front >= self.front_clear
                and min(left, right) >= self.minimum_turn_clearance
            ) or (progressed and front >= self.front_stop):
                boxed_for = now - self.boxed_started if self.boxed_started else 0.0
                self.get_logger().info(
                    f"boxed escape succeeded after {boxed_for:.1f} s; "
                    "resuming forward"
                )
                self.mode = "forward"
                self.front_blocked_since = None
                self.reverse_attempts = 0
                self.next_planned_turn_distance = (
                    self.path_length + self.planned_turn_distance
                )
            elif now - self.probe_started > 4.0:
                if not progressed:
                    self.probe_index += 1
                self.probe_started = now
                self.probe_start_yaw = self.yaw
                self.probe_start_position = self.last_position
            idx = self.probe_index % len(candidates)
            lin_sign, ang_sign = candidates[idx]
            self.probe_command = (
                lin_sign * (self.reverse_speed if lin_sign < 0.0 else 0.05),
                ang_sign * 0.6 * self.angular_speed,
            )

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
                        self._enter_boxed(
                            now, "insufficient swept-corner turn clearance"
                        )

        if self.mode != mode_at_start:
            self.output_held_since = None
        # A silent CM (vetoing) counts as zero output, not as missing data.
        output_is_zero = (
            abs(self.last_output.linear.x) < 0.005
            and abs(self.last_output.angular.z) < 0.01
        ) or not self._fresh(self.output_time, self.output_timeout, now)
        active_motion_mode = self.mode in (
            "forward",
            "turning",
            "planned_turning",
            "reversing",
        )
        # In a passage the slim footprint plus scan noise makes CM flicker;
        # give it patience there so intermittent passes accumulate progress
        # through the door instead of triggering a retreat off it.
        hold_limit = 4.0 if passage_gap is not None else 1.2
        chain_warm = (
            self.motion_started
            and now - self.motion_start_time > max(2.0, self.output_timeout)
        )
        if output_is_zero and active_motion_mode and chain_warm:
            # A Collision Monitor flickering at a footprint boundary can pass
            # a single nonzero message every couple of seconds; that must not
            # reset the hold detector, or a blocked robot waits forever
            # (observed for 40 s against a table leg on 2026-08-14).
            self.output_active_since = None
            if self.output_held_since is None:
                self.output_held_since = now
            elif now - self.output_held_since > hold_limit:
                blocked_mode = self.mode
                self.output_held_since = None
                if blocked_mode == "forward":
                    self._begin_escape(
                        now,
                        left_turn_clearance,
                        right_turn_clearance,
                        rear_center,
                        "collision monitor blocked forward motion",
                    )
                elif blocked_mode in ("turning", "planned_turning"):
                    if not self._start_reverse(
                        now,
                        rear_center,
                        "collision monitor blocked turning motion",
                    ):
                        self._enter_boxed(
                            now, "collision monitor blocked turning motion"
                        )
                else:
                    self._enter_boxed(
                        now, "collision monitor blocked reverse motion"
                    )
        elif active_motion_mode:
            # Clear the hold only after the output has stayed nonzero for a
            # sustained interval; isolated passed messages keep the timer.
            if self.output_active_since is None:
                self.output_active_since = now
            if now - self.output_active_since >= 0.5:
                self.output_held_since = None
        else:
            self.output_active_since = None
            self.output_held_since = None

        if self.mode == "forward":
            if passage_gap is not None:
                command_linear = min(self.passage_speed, self.linear_speed)
                command_angular = max(
                    -self.gap_steer_cap,
                    min(
                        self.gap_steer_cap,
                        self.gap_steer_gain * passage_gap[0],
                    ),
                )
            else:
                command_linear = self.linear_speed
                command_angular = 0.0
        elif self.mode == "front_waiting":
            command_linear = 0.0
            command_angular = 0.0
        elif self.mode == "reversing":
            command_linear = -self.reverse_speed
            command_angular = 0.0
        elif self.mode == "boxed_probe":
            command_linear, command_angular = self.probe_command
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
