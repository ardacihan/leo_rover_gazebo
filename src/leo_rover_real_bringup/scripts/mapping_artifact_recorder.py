#!/usr/bin/env python3

"""Publish the corrected SLAM path and save the final map with a path overlay."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

from mapping_artifacts import encode_png_rgb, render_path_overlay


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MappingArtifactRecorder(Node):
    """Record map-frame poses and persist occupancy/path artifacts on request or exit."""

    def __init__(self):
        super().__init__("mapping_artifact_recorder")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/exploration_path")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("sample_period", 0.5)
        self.declare_parameter("minimum_translation", 0.02)
        self.declare_parameter("minimum_rotation", math.radians(2.0))
        self.declare_parameter("output_directory", "")
        self.declare_parameter("artifact_prefix", "leo_room")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.minimum_translation = max(
            float(self.get_parameter("minimum_translation").value), 0.0
        )
        self.minimum_rotation = max(
            float(self.get_parameter("minimum_rotation").value), 0.0
        )
        configured_directory = str(self.get_parameter("output_directory").value)
        self.output_directory = (
            Path(configured_directory).expanduser()
            if configured_directory
            else Path.home() / "leo_maps"
        )
        unsafe_prefix = str(self.get_parameter("artifact_prefix").value)
        self.artifact_prefix = re.sub(r"[^A-Za-z0-9_.-]+", "_", unsafe_prefix).strip("._")
        if not self.artifact_prefix:
            self.artifact_prefix = "leo_room"
        self.run_stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_map = None
        self.path = PathMessage()
        self.path.header.frame_id = self.map_frame
        self.samples = []
        self.path_length = 0.0
        self.last_saved_signature = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.path_publisher = self.create_publisher(
            PathMessage,
            str(self.get_parameter("path_topic").value),
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            transient_qos,
        )
        self.create_service(Trigger, "save_mapping_artifacts", self._save_callback)
        self.create_timer(
            max(float(self.get_parameter("sample_period").value), 0.1),
            self._sample_pose,
        )
        self.get_logger().info(
            f"recording {self.map_frame} <- {self.base_frame} path; "
            f"artifacts will be saved under {self.output_directory}"
        )

    def _map_callback(self, msg):
        self.latest_map = msg

    def _sample_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except tf2_ros.TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x = float(translation.x)
        y = float(translation.y)
        yaw = quaternion_yaw(rotation)
        if self.samples:
            previous = self.samples[-1]
            distance = math.hypot(x - previous[1], y - previous[2])
            heading_change = abs(math.atan2(
                math.sin(yaw - previous[3]), math.cos(yaw - previous[3])
            ))
            if distance < self.minimum_translation and heading_change < self.minimum_rotation:
                return
            self.path_length += distance

        stamp_seconds = self.get_clock().now().nanoseconds / 1.0e9
        self.samples.append((stamp_seconds, x, y, yaw))
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rotation
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    @staticmethod
    def _map_origin(map_message):
        origin = map_message.info.origin
        return (
            (float(origin.position.x), float(origin.position.y)),
            quaternion_yaw(origin.orientation),
        )

    def save(self, force=False):
        if self.latest_map is None:
            raise RuntimeError("no occupancy map has been received")
        if not self.samples:
            raise RuntimeError("no map-to-base pose has been recorded")
        signature = (
            len(self.samples),
            int(self.latest_map.header.stamp.sec),
            int(self.latest_map.header.stamp.nanosec),
            int(self.latest_map.info.width),
            int(self.latest_map.info.height),
        )
        if not force and signature == self.last_saved_signature:
            return None

        self.output_directory.mkdir(parents=True, exist_ok=True)
        stem = f"{self.artifact_prefix}_{self.run_stamp}"
        base_path = self.output_directory / stem
        map_message = self.latest_map
        width = int(map_message.info.width)
        height = int(map_message.info.height)
        resolution = float(map_message.info.resolution)
        origin_xy, origin_yaw = self._map_origin(map_message)
        values = np.asarray(map_message.data, dtype=np.int16).reshape(height, width)

        pgm = np.full(values.shape, 205, dtype=np.uint8)
        pgm[(values >= 0) & (values <= 25)] = 254
        pgm[values >= 65] = 0
        pgm_path = base_path.with_suffix(".pgm")
        with pgm_path.open("wb") as stream:
            stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            stream.write(np.flipud(pgm).tobytes())

        yaml_path = base_path.with_suffix(".yaml")
        yaml_path.write_text(
            "\n".join([
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution:.9g}",
                f"origin: [{origin_xy[0]:.9g}, {origin_xy[1]:.9g}, {origin_yaw:.9g}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]),
            encoding="utf-8",
        )

        csv_path = self.output_directory / f"{stem}_path.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("time_seconds", "x_map_m", "y_map_m", "yaw_rad"))
            writer.writerows(self.samples)

        overlay = render_path_overlay(
            map_message.data,
            width,
            height,
            resolution,
            origin_xy,
            origin_yaw,
            [(sample[1], sample[2]) for sample in self.samples],
        )
        overlay_path = self.output_directory / f"{stem}_path.png"
        overlay_path.write_bytes(encode_png_rgb(overlay))

        summary_path = self.output_directory / f"{stem}_summary.json"
        summary_path.write_text(
            json.dumps({
                "saved_at_utc": datetime.now(timezone.utc).isoformat(),
                "map_frame": self.map_frame,
                "base_frame": self.base_frame,
                "map_width_cells": width,
                "map_height_cells": height,
                "resolution_m": resolution,
                "known_cells": int(np.count_nonzero(values >= 0)),
                "path_samples": len(self.samples),
                "path_length_m": self.path_length,
                "start_xy_m": list(self.samples[0][1:3]),
                "end_xy_m": list(self.samples[-1][1:3]),
            }, indent=2) + "\n",
            encoding="utf-8",
        )
        self.last_saved_signature = signature
        if rclpy.ok():
            self.get_logger().info(
                f"saved map and {self.path_length:.2f} m path to {overlay_path}"
            )
        return overlay_path

    def _save_callback(self, _request, response):
        try:
            path = self.save(force=True)
            response.success = True
            response.message = str(path)
        except (OSError, RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MappingArtifactRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.save()
            except (OSError, RuntimeError, ValueError) as error:
                if rclpy.ok():
                    node.get_logger().warning(
                        f"final artifact save skipped: {error}"
                    )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
