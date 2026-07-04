#!/usr/bin/env python3
"""
One-shot saver for the final shared mapping outputs.

Run while the demo is up:
  ros2 run multi_robot_shared_mapping save_shared_outputs

Waits for the latest messages, then writes into output_dir (default: maps/):
- <map_name>.pgm + <map_name>.yaml       nav2-style occupancy map of /shared_map
- <landmarks_name>.yaml                  merged AprilTag landmarks from
                                         /shared/apriltag_landmarks_data
- alignment_debug.json                   latest /alignment_debug_json payload
                                         plus saved /alignment_confidence

Accepted transform is recorded from /map_based_transform/leo2_to_leo1 when
available, otherwise /estimated_transform/leo2_to_leo1 (evaluation only).
No ground truth is used.
"""

from __future__ import annotations

import json
import math
import os
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32, String


class SaveSharedOutputs(Node):
    def __init__(self):
        super().__init__("save_shared_outputs")

        self.declare_parameter("output_dir", "maps")
        self.declare_parameter("map_name", "shared_office_map")
        self.declare_parameter("landmarks_name", "apriltag_landmarks_merged")
        self.declare_parameter("timeout_sec", 20.0)
        # Standard nav2 map_saver thresholds (occupancy in percent).
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("occupied_threshold", 65)

        self.shared_map: Optional[OccupancyGrid] = None
        self.shared_landmarks: List[Dict] = []
        self.alignment_debug: Optional[dict] = None
        self.alignment_confidence: Optional[float] = None
        self.map_transform: Optional[Tuple[float, float, float]] = None
        self.tag_transform: Optional[Tuple[float, float, float]] = None
        self.done = False

        self.create_subscription(OccupancyGrid, "/shared_map", self._map_cb, 10)
        self.create_subscription(String, "/shared/apriltag_landmarks_data", self._shared_lm_cb, 10)
        self.create_subscription(String, "/alignment_debug_json", self._debug_cb, 10)
        self.create_subscription(Float32, "/alignment_confidence", self._conf_cb, 10)
        self.create_subscription(
            TransformStamped, "/estimated_transform/leo2_to_leo1",
            lambda msg: self._transform_cb(msg, "tag"), 10,
        )
        self.create_subscription(
            TransformStamped, "/map_based_transform/leo2_to_leo1",
            lambda msg: self._transform_cb(msg, "map"), 10,
        )

        self._start_sec = self._now_sec()
        self.timer = self.create_timer(0.5, self._try_save)
        self.get_logger().info("waiting for /shared_map and landmark data...")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg: OccupancyGrid):
        self.shared_map = msg

    def _shared_lm_cb(self, msg: String):
        self.shared_landmarks = json.loads(msg.data)

    def _debug_cb(self, msg: String):
        self.alignment_debug = json.loads(msg.data)

    def _conf_cb(self, msg: Float32):
        self.alignment_confidence = float(msg.data)

    def _transform_cb(self, msg: TransformStamped, source: str):
        yaw = 2.0 * math.atan2(msg.transform.rotation.z, msg.transform.rotation.w)
        value = (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            yaw,
        )
        if source == "map":
            self.map_transform = value
        else:
            self.tag_transform = value

    def _try_save(self):
        if self.done:
            return

        elapsed = self._now_sec() - self._start_sec
        timed_out = elapsed > float(self.get_parameter("timeout_sec").value)
        have_map = self.shared_map is not None

        if not have_map and not timed_out:
            return
        if not have_map:
            self.get_logger().error("timeout: /shared_map never received, nothing saved")
            self.done = True
            rclpy.shutdown()
            return
        # Give optional topics a moment to arrive alongside the map.
        if not timed_out and elapsed < 5.0 and (
            not self.shared_landmarks or self.alignment_debug is None
        ):
            return

        output_dir = str(self.get_parameter("output_dir").value)
        os.makedirs(output_dir, exist_ok=True)

        map_name = str(self.get_parameter("map_name").value)
        self._save_occupancy_map(output_dir, map_name)
        self._save_landmarks(output_dir)
        self._save_alignment_debug(output_dir)

        self.done = True
        rclpy.shutdown()

    def _save_alignment_debug(self, output_dir: str):
        if self.alignment_debug is None:
            return
        path = os.path.join(output_dir, "alignment_debug.json")
        payload = dict(self.alignment_debug)
        if self.alignment_confidence is not None:
            payload["saved_confidence"] = self.alignment_confidence
        with open(path, "w") as out:
            json.dump(payload, out, indent=2)
        self.get_logger().info(f"saved alignment debug: {path}")

    def _save_occupancy_map(self, output_dir: str, map_name: str):
        grid = self.shared_map
        free_thr = int(self.get_parameter("free_threshold").value)
        occ_thr = int(self.get_parameter("occupied_threshold").value)

        pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
        yaml_path = os.path.join(output_dir, f"{map_name}.yaml")

        width = grid.info.width
        height = grid.info.height
        # PGM rows go top to bottom; occupancy grid rows go bottom to top.
        rows = []
        for iy in range(height - 1, -1, -1):
            row = bytearray(width)
            base = iy * width
            for ix in range(width):
                value = grid.data[base + ix]
                if value < 0:
                    row[ix] = 205  # unknown
                elif value >= occ_thr:
                    row[ix] = 0  # occupied
                elif value <= free_thr:
                    row[ix] = 254  # free
                else:
                    row[ix] = 205
            rows.append(bytes(row))

        with open(pgm_path, "wb") as pgm:
            pgm.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            for row in rows:
                pgm.write(row)

        with open(yaml_path, "w") as meta:
            meta.write(
                f"image: {map_name}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {grid.info.resolution}\n"
                f"origin: [{grid.info.origin.position.x}, "
                f"{grid.info.origin.position.y}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: {occ_thr / 100.0}\n"
                f"free_thresh: {free_thr / 100.0}\n"
            )
        self.get_logger().info(f"saved occupancy map: {pgm_path} + {yaml_path}")

    def _save_landmarks(self, output_dir: str):
        landmarks_name = str(self.get_parameter("landmarks_name").value)
        path = os.path.join(output_dir, f"{landmarks_name}.yaml")

        merged = self.shared_landmarks
        if not merged:
            self.get_logger().warn("no /shared/apriltag_landmarks_data; landmark YAML skipped")
            return

        with open(path, "w") as out:
            out.write("# Merged AprilTag landmarks in leo1/map frame.\n")
            source = "map_based" if self.map_transform else (
                "tag_based" if self.tag_transform else "none (leo1 only)"
            )
            out.write(f"# leo2 -> leo1 transform source: {source}\n")
            out.write("landmarks:\n")
            for lm in merged:
                out.write(
                    f"  - tag_id: {lm['tag_id']}\n"
                    f"    x: {lm['x']}\n"
                    f"    y: {lm['y']}\n"
                    f"    yaw: {lm['yaw']}\n"
                    f"    confidence: {lm['confidence']}\n"
                    f"    observation_count: {lm['observation_count']}\n"
                )
        self.get_logger().info(f"saved {len(merged)} merged landmarks: {path}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveSharedOutputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
