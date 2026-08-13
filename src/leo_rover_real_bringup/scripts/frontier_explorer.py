#!/usr/bin/env python3

"""Drive to map frontiers until the room is mapped.

Commands go out on `/cmd_vel_request` only, so the fail-closed gate and the
Collision Monitor stay in the path exactly as they are for the bounded
explorer. This node decides *where* to go; those two decide whether the motion
is allowed.

It publishes `/exploration_status` so the incident recorder can label captures
with what the rover was trying to do at the moment something went wrong.
"""

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from frontier_policy import (
    distance_to,
    heading_error,
    select_target,
    should_abandon,
    turn_command,
)
from map_coverage import cluster_frontiers, cluster_to_map_xy, coverage_stats, frontier_mask


class FrontierExplorer(Node):
    """Seek unexplored space, yielding to the safety chain for permission."""

    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan_collision_fused")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("battery_topic", "/rob_2/firmware/battery_averaged")
        self.declare_parameter("cmd_vel_request_topic", "/cmd_vel_request")
        self.declare_parameter("cmd_vel_output_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("run_duration", 600.0)
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("turn_gain", 1.2)
        self.declare_parameter("heading_tolerance", 0.25)
        self.declare_parameter("arrival_radius", 0.45)
        self.declare_parameter("target_time_budget", 60.0)
        self.declare_parameter("stall_timeout", 10.0)
        self.declare_parameter("minimum_progress", 0.12)
        self.declare_parameter("blacklist_radius", 0.6)
        self.declare_parameter("minimum_cluster_cells", 6)
        self.declare_parameter("front_stop_distance", 0.55)
        self.declare_parameter("front_clear_distance", 0.70)
        self.declare_parameter("minimum_turn_clearance", 0.40)
        self.declare_parameter("minimum_battery_voltage", 10.20)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("battery_timeout", 2.0)
        self.declare_parameter("completion_confirmations", 3)

        get = self.get_parameter
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.run_duration = float(get("run_duration").value)
        self.linear_speed = min(abs(float(get("linear_speed").value)), 0.10)
        self.max_angular = min(abs(float(get("max_angular_speed").value)), 0.30)
        self.turn_gain = float(get("turn_gain").value)
        self.heading_tolerance = float(get("heading_tolerance").value)
        self.arrival_radius = float(get("arrival_radius").value)
        self.target_time_budget = float(get("target_time_budget").value)
        self.stall_timeout = float(get("stall_timeout").value)
        self.minimum_progress = float(get("minimum_progress").value)
        self.blacklist_radius = float(get("blacklist_radius").value)
        self.minimum_cluster_cells = int(get("minimum_cluster_cells").value)
        self.front_stop = float(get("front_stop_distance").value)
        self.front_clear = float(get("front_clear_distance").value)
        self.minimum_turn_clearance = float(get("minimum_turn_clearance").value)
        self.minimum_battery = float(get("minimum_battery_voltage").value)
        self.scan_timeout = float(get("scan_timeout").value)
        self.odom_timeout = float(get("odom_timeout").value)
        self.battery_timeout = float(get("battery_timeout").value)
        self.completion_confirmations = int(get("completion_confirmations").value)

        self.cmd_output_topic = str(get("cmd_vel_output_topic").value)
        self.cmd_pub = self.create_publisher(
            Twist, str(get("cmd_vel_request_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/exploration_status", 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, str(get("map_topic").value), self._map_callback, map_qos
        )
        self.create_subscription(
            LaserScan, str(get("scan_topic").value), self._scan_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, str(get("odom_topic").value), self._odom_callback, 10
        )
        self.create_subscription(
            Float32, str(get("battery_topic").value), self._battery_callback,
            qos_profile_sensor_data
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        now = time.monotonic()
        self.start_time = now
        self.map = None
        self.scan = None
        self.scan_time = None
        self.odom = None
        self.odom_time = None
        self.battery = None
        self.battery_time = None
        self.target = None
        self.target_started = now
        self.target_start_xy = None
        self.stall_since = now
        self.best_distance = None
        self.blacklist = []
        self.mode = "acquiring"
        self.finish_reason = None
        self.path_length = 0.0
        self.last_xy = None
        self.empty_map_checks = 0
        self.clusters = []

        self.timer = self.create_timer(0.1, self._tick)
        self.create_timer(4.0, self._report)
        self.get_logger().info(
            f"frontier explorer ready: linear={self.linear_speed:.2f} m/s, "
            f"angular<={self.max_angular:.2f} rad/s, "
            f"duration={self.run_duration:.0f} s; stops when no frontier remains"
        )

    # ---------------- callbacks ----------------
    def _map_callback(self, msg):
        self.map = msg

    def _scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def _odom_callback(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_xy is not None:
            self.path_length += math.hypot(xy[0] - self.last_xy[0],
                                           xy[1] - self.last_xy[1])
        self.last_xy = xy

    def _battery_callback(self, msg):
        self.battery = float(msg.data)
        self.battery_time = time.monotonic()

    # ---------------- helpers ----------------
    @staticmethod
    def _fresh(stamp, timeout, now):
        return stamp is not None and now - stamp <= timeout

    def _pose(self):
        """Rover pose in the map frame, or None while TF is unavailable."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y), yaw

    def _sector(self, lower_deg, upper_deg):
        """5th-percentile clearance over a base-frame sector of the fused scan."""
        scan = self.scan
        if scan is None:
            return 0.0
        values = []
        angle = float(scan.angle_min)
        lo, hi = math.radians(lower_deg), math.radians(upper_deg)
        for reading in scan.ranges:
            normalized = math.atan2(math.sin(angle), math.cos(angle))
            if lo <= normalized <= hi:
                if math.isinf(reading) and reading > 0.0:
                    values.append(float(scan.range_max))
                elif math.isfinite(reading) and scan.range_min <= reading <= scan.range_max:
                    values.append(float(reading))
            angle += float(scan.angle_increment)
        if len(values) < 5:
            return 0.0
        values.sort()
        return values[len(values) // 20]

    def _frontier_clusters(self):
        if self.map is None:
            return []
        info = self.map.info
        cells = np.asarray(self.map.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        clusters = cluster_frontiers(
            frontier_mask(cells), self.minimum_cluster_cells
        )
        out = []
        for count, row, col in clusters:
            x, y = cluster_to_map_xy(
                row, col, info.origin.position.x, info.origin.position.y,
                info.resolution
            )
            out.append((count, x, y))
        return out

    def _publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _status(self, extra=""):
        target = ("none" if self.target is None
                  else f"({self.target[0]:+.2f},{self.target[1]:+.2f})")
        msg = String()
        msg.data = (f"mode={self.mode} target={target} "
                    f"front={self._sector(-30, 30):.2f} "
                    f"path={self.path_length:.2f} {extra}").strip()
        self.status_pub.publish(msg)

    def _blocking_reason(self, now):
        if not self._fresh(self.scan_time, self.scan_timeout, now):
            return "stale fused scan"
        if not self._fresh(self.odom_time, self.odom_timeout, now):
            return "stale odometry"
        if not self._fresh(self.battery_time, self.battery_timeout, now):
            return "stale battery telemetry"
        if self.battery is None or self.battery < self.minimum_battery:
            return f"battery below {self.minimum_battery:.2f} V"
        if self.map is None:
            return "no map yet"
        return None

    def _finish(self, reason):
        if self.finish_reason is None:
            self.finish_reason = reason
            self.mode = "stopping"
            self.get_logger().info(
                f"stopping: {reason}; path={self.path_length:.2f} m, "
                f"{len(self.blacklist)} targets abandoned"
            )
        self._publish()
        self._status(f"finished={reason}")

    # ---------------- main loop ----------------
    def _tick(self):
        now = time.monotonic()
        if self.finish_reason is not None:
            self._publish()
            return
        if now - self.start_time >= self.run_duration:
            self._finish("run duration reached")
            return
        reason = self._blocking_reason(now)
        if reason is not None:
            self.mode = "waiting"
            self._publish()
            self._status(f"blocked={reason}")
            return
        pose = self._pose()
        if pose is None:
            self.mode = "waiting"
            self._publish()
            self._status("blocked=no map->base transform")
            return
        xy, yaw = pose

        # Re-acquire whenever there is no target.
        if self.target is None:
            self.clusters = self._frontier_clusters()
            self.target = select_target(
                self.clusters, xy, self.blacklist, self.blacklist_radius,
                self.minimum_cluster_cells
            )
            if self.target is None:
                # Require repeated confirmation: a single map update can drop
                # frontiers briefly while SLAM rewrites cells after a loop
                # closure, and quitting on that would end the run early.
                self.empty_map_checks += 1
                if self.empty_map_checks >= self.completion_confirmations:
                    self._finish("map complete: no reachable frontiers remain")
                else:
                    self._publish()
                    self._status("verifying completion")
                return
            self.empty_map_checks = 0
            self.mode = "turning"
            self.target_started = now
            self.target_start_xy = xy
            self.stall_since = now
            self.best_distance = distance_to(self.target, xy)
            self.get_logger().info(
                f"target ({self.target[0]:+.2f}, {self.target[1]:+.2f}) m, "
                f"{self.best_distance:.2f} m away, "
                f"{len(self.clusters)} frontiers known"
            )

        distance = distance_to(self.target, xy)
        if distance + 0.05 < (self.best_distance or distance):
            self.best_distance = distance
            self.stall_since = now
        progress = (self.best_distance or distance)
        progress = max(0.0, distance_to(self.target, self.target_start_xy or xy) - progress)

        if distance <= self.arrival_radius:
            self.get_logger().info("target reached; re-acquiring")
            self.target = None
            return

        abandon = should_abandon(
            now - self.target_started, self.target_time_budget, progress,
            self.minimum_progress, now - self.stall_since, self.stall_timeout
        )
        if abandon is not None:
            self.get_logger().warn(
                f"abandoning target ({self.target[0]:+.2f}, "
                f"{self.target[1]:+.2f}): {abandon}"
            )
            self.blacklist.append(self.target)
            self._status(f"abandoned={abandon}")
            self.target = None
            self._publish()
            return

        front = self._sector(-30.0, 30.0)
        left = self._sector(25.0, 100.0)
        right = self._sector(-100.0, -25.0)
        error = heading_error(self.target, xy, yaw)

        # Rotating in place closes no distance, so the stall timer must not run
        # while the rover is still lining up. A half turn at the angular cap
        # takes over ten seconds, which is longer than the stall timeout: left
        # running, it abandons every target before ever driving at one.
        if abs(error) > self.heading_tolerance or front < self.front_stop:
            self.stall_since = now

        # Obstacle handling wins over goal seeking. Turning toward the freer
        # side rather than the target keeps the rover from grinding along a
        # wall that happens to lie between it and the frontier.
        if front < self.front_stop:
            self.mode = "avoiding"
            direction = 1.0 if left >= right else -1.0
            if max(left, right) < self.minimum_turn_clearance:
                self.blacklist.append(self.target)
                self.get_logger().warn("boxed in; abandoning target")
                self.target = None
                self._publish()
                return
            self._publish(0.0, direction * self.max_angular)
        elif abs(error) > self.heading_tolerance:
            self.mode = "turning"
            self._publish(0.0, turn_command(error, self.turn_gain, self.max_angular))
        else:
            self.mode = "driving"
            speed = self.linear_speed
            if front < self.front_clear:
                speed *= 0.5
            self._publish(speed, turn_command(error, self.turn_gain * 0.5,
                                              self.max_angular * 0.5, 0.0))
        self._status(f"dist={distance:.2f} err={math.degrees(error):+.0f}deg")

    def _report(self):
        if self.map is None:
            return
        cells = np.asarray(self.map.data, dtype=np.int16).reshape(
            self.map.info.height, self.map.info.width
        )
        unknown, free, occupied = coverage_stats(cells)
        self.get_logger().info(
            f"mode={self.mode} unknown={100*unknown:.1f}% free={100*free:.1f}% "
            f"occupied={100*occupied:.1f}% frontiers={len(self.clusters)} "
            f"path={self.path_length:.2f} m "
            f"battery={self.battery if self.battery else float('nan'):.2f} V"
        )

    def publish_final_zeros(self):
        for _ in range(20):
            self._publish()
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FrontierExplorer()
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
