#!/usr/bin/env python3

"""Capture RGB, depth and scan evidence whenever the rover refuses to move.

Answers "why did it stop there" after the fact. Every capture writes an
annotated panel plus a JSON sidecar holding the numbers the panel is drawn
from, and a continuous annotated video is written alongside so a capture can
be seen in the context of what led up to it.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String

from incident_triggers import (
    gate_block,
    monitor_veto,
    near_miss,
    sensor_conflict,
    should_capture,
)

PANEL_H = 480
FONT = cv2.FONT_HERSHEY_SIMPLEX


class IncidentRecorder(Node):
    """Watch the command chain and the sensors, and snapshot the bad moments."""

    def __init__(self):
        super().__init__("incident_recorder")
        self.declare_parameter("output_directory", "/home/jetson-04/leo_incidents")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("lidar_scan_topic", "/scan_lidar_base")
        self.declare_parameter("fused_scan_topic", "/scan_collision_fused")
        self.declare_parameter("camera_scan_topic", "/camera/scan_collision")
        self.declare_parameter("request_topic", "/cmd_vel_request")
        self.declare_parameter("raw_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/exploration_status")
        self.declare_parameter("battery_topic", "/rob_2/firmware/battery_averaged")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("minimum_capture_interval", 1.5)
        # Per-kind limits. A veto is a rare event worth catching every time,
        # while a camera/lidar disagreement is expected wherever a low obstacle
        # sits under the lidar plane, so at one shared interval the routine
        # disagreements would crowd out the rare vetoes.
        self.declare_parameter("interval_monitor_veto", 1.5)
        self.declare_parameter("interval_gate_block", 1.5)
        self.declare_parameter("interval_near_miss", 3.0)
        self.declare_parameter("interval_sensor_conflict", 8.0)
        self.declare_parameter("heartbeat_period", 10.0)
        self.declare_parameter("maximum_captures", 400)
        self.declare_parameter("near_miss_distance", 0.50)
        # Each frame composes three 640x480 panels and encodes them. The Jetson
        # has no headroom to spare while SLAM and depth fusion run, and starving
        # it stalls telemetry, so the video runs slow enough to stay cheap while
        # still showing what led up to a capture.
        self.declare_parameter("video_fps", 2.0)
        self.declare_parameter("record_video", True)

        get = self.get_parameter
        self.directory = str(get("output_directory").value)
        os.makedirs(self.directory, exist_ok=True)
        self.minimum_interval = float(get("minimum_capture_interval").value)
        self.heartbeat_period = float(get("heartbeat_period").value)
        self.intervals = {
            "monitor_veto": float(get("interval_monitor_veto").value),
            "gate_block": float(get("interval_gate_block").value),
            "near_miss": float(get("interval_near_miss").value),
            "sensor_conflict": float(get("interval_sensor_conflict").value),
            "heartbeat": self.heartbeat_period,
        }
        self.maximum_captures = int(get("maximum_captures").value)
        self.near_miss_distance = float(get("near_miss_distance").value)
        self.record_video = bool(get("record_video").value)
        self.video_fps = float(get("video_fps").value)

        self.rgb = None
        self.depth = None
        self.lidar = None
        self.fused = None
        self.camera_scan = None
        self.request = Twist()
        self.raw = Twist()
        self.output = Twist()
        self.output_time = None
        self.status = ""
        self.battery = float("nan")
        self.pose = (0.0, 0.0)
        self.last_capture = {}
        self.captures = 0
        self.writer = None
        self.session = time.strftime("%Y%m%d_%H%M%S")

        self.create_subscription(Image, str(get("rgb_topic").value),
                                 self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, str(get("depth_topic").value),
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(get("lidar_scan_topic").value),
                                 self._lidar_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(get("fused_scan_topic").value),
                                 self._fused_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(get("camera_scan_topic").value),
                                 self._camera_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, str(get("request_topic").value),
                                 self._request_cb, 10)
        self.create_subscription(Twist, str(get("raw_topic").value),
                                 self._raw_cb, 10)
        self.create_subscription(Twist, str(get("output_cmd_topic").value),
                                 self._output_cb, 10)
        self.create_subscription(String, str(get("status_topic").value),
                                 self._status_cb, 10)
        self.create_subscription(Float32, str(get("battery_topic").value),
                                 self._battery_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(get("odom_topic").value),
                                 self._odom_cb, 10)

        self.create_timer(0.2, self._evaluate)
        if self.record_video:
            self.create_timer(1.0 / max(self.video_fps, 1.0), self._video_frame)
        self.get_logger().info(
            f"incident recorder ready -> {self.directory} (session {self.session})"
        )

    # ---------------- callbacks ----------------
    def _rgb_cb(self, msg):
        self.rgb = msg

    def _depth_cb(self, msg):
        self.depth = msg

    def _lidar_cb(self, msg):
        self.lidar = msg

    def _fused_cb(self, msg):
        self.fused = msg

    def _camera_cb(self, msg):
        self.camera_scan = msg

    def _request_cb(self, msg):
        self.request = msg

    def _raw_cb(self, msg):
        self.raw = msg

    def _output_cb(self, msg):
        self.output = msg
        self.output_time = time.monotonic()

    def _status_cb(self, msg):
        self.status = msg.data

    def _battery_cb(self, msg):
        self.battery = float(msg.data)

    def _odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ---------------- geometry ----------------
    @staticmethod
    def _sector(scan, lower_deg, upper_deg):
        if scan is None:
            return 0.0
        values = []
        angle = float(scan.angle_min)
        lo, hi = math.radians(lower_deg), math.radians(upper_deg)
        for reading in scan.ranges:
            normalized = math.atan2(math.sin(angle), math.cos(angle))
            if lo <= normalized <= hi and math.isfinite(reading) \
                    and scan.range_min <= reading <= scan.range_max:
                values.append(float(reading))
            angle += float(scan.angle_increment)
        if len(values) < 5:
            return 0.0
        values.sort()
        return values[len(values) // 20]

    def _metrics(self):
        return {
            "front_fused": self._sector(self.fused, -30, 30),
            "front_lidar": self._sector(self.lidar, -30, 30),
            "front_camera": self._sector(self.camera_scan, -30, 30),
            "left_fused": self._sector(self.fused, 25, 100),
            "right_fused": self._sector(self.fused, -100, -25),
            "request_linear": float(self.request.linear.x),
            "request_angular": float(self.request.angular.z),
            "raw_linear": float(self.raw.linear.x),
            "raw_angular": float(self.raw.angular.z),
            "output_linear": float(self.output.linear.x),
            "output_angular": float(self.output.angular.z),
            "battery_v": self.battery,
            "pose_x": self.pose[0],
            "pose_y": self.pose[1],
            "status": self.status,
        }

    # ---------------- triggers ----------------
    def _evaluate(self):
        if self.captures >= self.maximum_captures:
            return
        now = time.monotonic()
        m = self._metrics()
        kinds = []
        if monitor_veto(m["raw_linear"], m["output_linear"]):
            kinds.append("monitor_veto")
        if gate_block(m["request_linear"], m["request_angular"],
                      m["raw_linear"], m["raw_angular"]):
            kinds.append("gate_block")
        if near_miss(m["front_fused"], self.near_miss_distance):
            kinds.append("near_miss")
        if sensor_conflict(m["front_lidar"], m["front_camera"]):
            kinds.append("sensor_conflict")
        if should_capture(now, self.last_capture, "heartbeat",
                          self.heartbeat_period):
            kinds.append("heartbeat")
        for kind in kinds:
            interval = self.intervals.get(kind, self.minimum_interval)
            if should_capture(now, self.last_capture, kind, interval):
                self.last_capture[kind] = now
                self._capture(kind, m)

    # ---------------- rendering ----------------
    def _rgb_image(self):
        if self.rgb is None:
            return np.zeros((PANEL_H, 640, 3), dtype=np.uint8)
        msg = self.rgb
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr[: msg.height * msg.width * 3].reshape(msg.height, msg.width, 3)
        img = arr if msg.encoding == "bgr8" else arr[:, :, ::-1]
        return cv2.resize(np.ascontiguousarray(img), (640, PANEL_H))

    def _depth_image(self):
        if self.depth is None:
            return np.zeros((PANEL_H, 640, 3), dtype=np.uint8)
        msg = self.depth
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        ).astype(np.float32)
        arr = np.clip(arr, 0, 4000) / 4000.0 * 255.0
        colored = cv2.applyColorMap(arr.astype(np.uint8), cv2.COLORMAP_TURBO)
        colored[arr == 0] = (0, 0, 0)
        return cv2.resize(colored, (640, PANEL_H))

    def _scan_image(self, metrics, size=PANEL_H, span=4.0):
        """Top-down plot: lidar in grey, camera in cyan, fused in white."""
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        centre = size // 2
        scale = (size / 2.0) / span
        for radius_m in (1.0, 2.0, 3.0):
            cv2.circle(canvas, (centre, centre), int(radius_m * scale),
                       (40, 40, 40), 1)
            cv2.putText(canvas, f"{radius_m:.0f}m",
                        (centre + int(radius_m * scale) - 22, centre - 4),
                        FONT, 0.35, (90, 90, 90), 1)

        def draw(scan, colour, radius=1):
            if scan is None:
                return
            angle = float(scan.angle_min)
            for reading in scan.ranges:
                if math.isfinite(reading) and scan.range_min <= reading <= span:
                    # Rover x forward drawn up the image, y left drawn left.
                    px = int(centre - reading * math.sin(angle) * scale)
                    py = int(centre - reading * math.cos(angle) * scale)
                    if 0 <= px < size and 0 <= py < size:
                        cv2.circle(canvas, (px, py), radius, colour, -1)
                angle += float(scan.angle_increment)

        draw(self.lidar, (120, 120, 120))
        draw(self.camera_scan, (255, 255, 0))
        draw(self.fused, (255, 255, 255))

        half = int(0.22 * scale)
        cv2.rectangle(canvas, (centre - half, centre - half),
                      (centre + half, centre + half), (0, 200, 0), 1)
        for deg in (-30, 30):
            a = math.radians(deg)
            cv2.line(canvas, (centre, centre),
                     (int(centre - span * math.sin(a) * scale),
                      int(centre - span * math.cos(a) * scale)),
                     (0, 140, 255), 1)
        cv2.putText(canvas, "lidar", (8, size - 44), FONT, 0.4, (120, 120, 120), 1)
        cv2.putText(canvas, "camera", (8, size - 28), FONT, 0.4, (255, 255, 0), 1)
        cv2.putText(canvas, "fused", (8, size - 12), FONT, 0.4, (255, 255, 255), 1)
        cv2.putText(canvas, f"front {metrics['front_fused']:.2f}m",
                    (8, 18), FONT, 0.45, (0, 200, 255), 1)
        return canvas

    def _panel(self, kind, metrics):
        panel = np.hstack([self._rgb_image(), self._depth_image(),
                           self._scan_image(metrics)])
        header = np.zeros((96, panel.shape[1], 3), dtype=np.uint8)
        colour = (0, 0, 255) if kind != "heartbeat" else (0, 200, 0)
        lines = [
            f"TRIGGER: {kind}    t={time.strftime('%H:%M:%S')}    "
            f"battery={metrics['battery_v']:.2f}V",
            f"cmd request=({metrics['request_linear']:+.3f}, "
            f"{metrics['request_angular']:+.3f})  "
            f"gate=({metrics['raw_linear']:+.3f}, {metrics['raw_angular']:+.3f})"
            f"  monitor=({metrics['output_linear']:+.3f}, "
            f"{metrics['output_angular']:+.3f})",
            f"front fused={metrics['front_fused']:.2f} "
            f"lidar={metrics['front_lidar']:.2f} "
            f"camera={metrics['front_camera']:.2f}  "
            f"left={metrics['left_fused']:.2f} right={metrics['right_fused']:.2f}",
            f"explorer: {metrics['status'][:110]}",
        ]
        for index, text in enumerate(lines):
            cv2.putText(header, text, (10, 22 + index * 20), FONT, 0.45,
                        colour if index == 0 else (230, 230, 230), 1)
        return np.vstack([header, panel])

    def _capture(self, kind, metrics):
        stamp = time.strftime("%H%M%S") + f"_{int(time.time()*1000)%1000:03d}"
        base = os.path.join(self.directory, f"{self.session}_{stamp}_{kind}")
        cv2.imwrite(base + ".png", self._panel(kind, metrics))
        payload = dict(metrics)
        payload["trigger"] = kind
        payload["wall_clock"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        with open(base + ".json", "w") as handle:
            json.dump(payload, handle, indent=2)
        self.captures += 1
        if kind != "heartbeat":
            self.get_logger().warn(
                f"capture {kind}: front={metrics['front_fused']:.2f} m "
                f"request={metrics['request_linear']:+.3f} -> "
                f"monitor={metrics['output_linear']:+.3f} -> {base}.png"
            )

    def _video_frame(self):
        if self.rgb is None:
            return
        frame = self._panel("live", self._metrics())
        if self.writer is None:
            path = os.path.join(self.directory, f"{self.session}_run.mp4")
            self.writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"), self.video_fps,
                (frame.shape[1], frame.shape[0])
            )
            self.get_logger().info(f"recording annotated video -> {path}")
        self.writer.write(frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f"{self.captures} captures written")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IncidentRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
