#!/usr/bin/env python3
"""
Consistency-aware occupancy-grid merger for two robots.

Uses only the ACCEPTED alignment transform (/map_based_transform/leo2_to_leo1
in map/hybrid, /estimated_transform in tag-only mode). Candidate transforms
never corrupt /shared_map.

Publishes:
- /shared_map           accepted stable merge (optionally cleaned)
- /shared_map_raw       raw accepted merge
- /shared_map_cleaned   visualization cleanup of raw
- /shared_map_candidate preview using latest candidate transform

When alignment confidence is low or local map quality is poor, leo2 data is
withheld and the previous accepted /shared_map is republished if available.
"""

import json
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Header, String

from multi_robot_shared_mapping.map_postprocess import clean_occupancy_grid
from multi_robot_shared_mapping.map_quality import LocalMapQualityTracker


class SharedMapMerger(Node):
    def __init__(self):
        super().__init__("shared_map_merger")

        self.declare_parameter("map1_topic", "/leo1/map")
        self.declare_parameter("map2_topic", "/leo2/map")
        self.declare_parameter("shared_map_topic", "/shared_map")
        self.declare_parameter("shared_map_raw_topic", "/shared_map_raw")
        self.declare_parameter("shared_map_cleaned_topic", "/shared_map_cleaned")
        self.declare_parameter("shared_map_candidate_topic", "/shared_map_candidate")
        self.declare_parameter("shared_frame_id", "leo1/map")
        self.declare_parameter("alignment_mode", "fixed")
        self.declare_parameter("estimated_transform_topic", "/estimated_transform/leo2_to_leo1")
        self.declare_parameter("map_transform_topic", "/map_based_transform/leo2_to_leo1")
        self.declare_parameter("candidate_transform_topic", "/alignment_candidate_transform")
        self.declare_parameter("confidence_topic", "/alignment_confidence")
        self.declare_parameter("min_alignment_confidence", 0.5)
        self.declare_parameter("use_cleaned_shared_map", True)
        self.declare_parameter("min_local_map_quality", 0.35)

        self.declare_parameter("robot2_to_shared_x", 0.0)
        self.declare_parameter("robot2_to_shared_y", 0.0)
        self.declare_parameter("robot2_to_shared_yaw", 0.0)
        self.declare_parameter("occupied_threshold", 50)

        self.map1: Optional[OccupancyGrid] = None
        self.map2: Optional[OccupancyGrid] = None
        self.accepted_valid = False
        self.accepted_dx = 0.0
        self.accepted_dy = 0.0
        self.accepted_yaw = 0.0
        self.candidate_dx = 0.0
        self.candidate_dy = 0.0
        self.candidate_yaw = 0.0
        self.candidate_valid = False
        self.alignment_confidence: Optional[float] = None
        self.confidence_level: str = "low"
        self.exploration_allowed: bool = False
        self.last_accepted_grid: Optional[OccupancyGrid] = None
        self.waiting_logged = False
        self.low_confidence_logged = False
        self._last_logged_transform = None

        self.quality_leo1 = LocalMapQualityTracker()
        self.quality_leo2 = LocalMapQualityTracker()

        map1_topic = self.get_parameter("map1_topic").value
        map2_topic = self.get_parameter("map2_topic").value
        mode = self._alignment_mode()

        self.create_subscription(OccupancyGrid, map1_topic, self._map1_cb, 10)
        self.create_subscription(OccupancyGrid, map2_topic, self._map2_cb, 10)

        self.shared_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("shared_map_topic").value), 10
        )
        self.raw_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("shared_map_raw_topic").value), 10
        )
        self.cleaned_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("shared_map_cleaned_topic").value), 10
        )
        self.candidate_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("shared_map_candidate_topic").value), 10
        )
        self.timer = self.create_timer(1.0, self._publish_all)

        if mode in ("tag", "map", "hybrid"):
            topic = str(self.get_parameter("map_transform_topic").value)
            self.create_subscription(TransformStamped, topic, self._accepted_cb, 10)
            cand = str(self.get_parameter("candidate_transform_topic").value)
            self.create_subscription(TransformStamped, cand, self._candidate_cb, 10)
            conf = str(self.get_parameter("confidence_topic").value)
            self.create_subscription(Float32, conf, self._confidence_cb, 10)
            self.create_subscription(String, "/alignment_debug_json", self._debug_cb, 10)

        self.get_logger().info(
            f"shared_map_merger mode={mode}; publishes accepted map only"
        )

    def _alignment_mode(self) -> str:
        mode = str(self.get_parameter("alignment_mode").value)
        return "tag" if mode == "estimated" else mode

    def _map1_cb(self, msg: OccupancyGrid):
        self.map1 = msg
        self.quality_leo1.update(
            msg.data, msg.info.width, msg.info.height,
            msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
            int(self.get_parameter("occupied_threshold").value),
        )

    def _map2_cb(self, msg: OccupancyGrid):
        self.map2 = msg
        self.quality_leo2.update(
            msg.data, msg.info.width, msg.info.height,
            msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
            int(self.get_parameter("occupied_threshold").value),
        )

    def _accepted_cb(self, msg: TransformStamped):
        qz, qw = msg.transform.rotation.z, msg.transform.rotation.w
        self.accepted_dx = float(msg.transform.translation.x)
        self.accepted_dy = float(msg.transform.translation.y)
        self.accepted_yaw = 2.0 * math.atan2(qz, qw)
        self.accepted_valid = True
        self.waiting_logged = False
        current = (self.accepted_dx, self.accepted_dy, self.accepted_yaw)
        if current != self._last_logged_transform:
            self._last_logged_transform = current
            self.get_logger().info(
                f"accepted transform dx={self.accepted_dx:.3f} "
                f"dy={self.accepted_dy:.3f} yaw={self.accepted_yaw:.3f}"
            )

    def _candidate_cb(self, msg: TransformStamped):
        qz, qw = msg.transform.rotation.z, msg.transform.rotation.w
        self.candidate_dx = float(msg.transform.translation.x)
        self.candidate_dy = float(msg.transform.translation.y)
        self.candidate_yaw = 2.0 * math.atan2(qz, qw)
        self.candidate_valid = True

    def _confidence_cb(self, msg: Float32):
        self.alignment_confidence = float(msg.data)

    def _debug_cb(self, msg: String):
        data = json.loads(msg.data)
        self.confidence_level = str(data.get("confidence_level", "low"))
        self.exploration_allowed = bool(data.get("exploration_allowed", False))

    def _fusion_allowed(self) -> bool:
        mode = self._alignment_mode()
        if mode == "fixed":
            return True
        if not self.accepted_valid:
            if not self.waiting_logged:
                self.waiting_logged = True
                self.get_logger().warn(
                    "waiting for valid alignment transform; merging leo1 map only"
                )
            return False
        threshold = float(self.get_parameter("min_alignment_confidence").value)
        level_ok = self.confidence_level in ("medium", "high")
        conf_ok = (
            self.alignment_confidence is not None
            and self.alignment_confidence >= threshold
        )
        if not level_ok and not conf_ok:
            if not self.low_confidence_logged:
                self.low_confidence_logged = True
                self.get_logger().warn(
                    f"Alignment confidence level={self.confidence_level}; "
                    f"withholding leo2 fusion (exploration_allowed="
                    f"{self.exploration_allowed})."
                )
            return False
        min_q = float(self.get_parameter("min_local_map_quality").value)
        if self.quality_leo1.quality < min_q or self.quality_leo2.quality < min_q:
            return False
        self.low_confidence_logged = False
        return True

    def _transform(self, use_candidate: bool = False) -> Optional[Tuple[float, float, float]]:
        if self._alignment_mode() == "fixed":
            return (
                float(self.get_parameter("robot2_to_shared_x").value),
                float(self.get_parameter("robot2_to_shared_y").value),
                float(self.get_parameter("robot2_to_shared_yaw").value),
            )
        if use_candidate and self.candidate_valid:
            return self.candidate_dx, self.candidate_dy, self.candidate_yaw
        if not self._fusion_allowed():
            return None
        return self.accepted_dx, self.accepted_dy, self.accepted_yaw

    def _transform_point(
        self, x: float, y: float, transform: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        return c * x - s * y + tx, s * x + c * y + ty

    def _merge_maps(
        self, include_leo2: bool, transform: Optional[Tuple[float, float, float]]
    ) -> Optional[OccupancyGrid]:
        if self.map1 is None:
            return None

        maps = [(self.map1, False)]
        if include_leo2 and self.map2 is not None and transform is not None:
            maps.append((self.map2, True))

        resolution = min(m.info.resolution for m, _ in maps)
        bounds = []
        for grid, is_r2 in maps:
            bound = self._map_bounds(grid, is_r2, transform)
            if bound is None:
                return None
            bounds.append(bound)

        min_x = min(b[0] for b in bounds)
        min_y = min(b[1] for b in bounds)
        max_x = max(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)
        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))

        shared = OccupancyGrid()
        shared.header = Header()
        shared.header.stamp = self.get_clock().now().to_msg()
        shared.header.frame_id = str(self.get_parameter("shared_frame_id").value)
        shared.info.resolution = resolution
        shared.info.width = width
        shared.info.height = height
        shared.info.origin.position.x = min_x
        shared.info.origin.position.y = min_y
        shared.info.origin.orientation.w = 1.0
        shared.data = [-1] * (width * height)

        threshold = int(self.get_parameter("occupied_threshold").value)
        for grid, is_r2 in maps:
            for iy in range(grid.info.height):
                for ix in range(grid.info.width):
                    value = grid.data[iy * grid.info.width + ix]
                    if value < 0:
                        continue
                    x, y = self._grid_to_world(grid, ix, iy)
                    if is_r2:
                        x, y = self._transform_point(x, y, transform)
                    sx = int(math.floor((x - min_x) / resolution))
                    sy = int(math.floor((y - min_y) / resolution))
                    if 0 <= sx < width and 0 <= sy < height:
                        idx = sy * width + sx
                        shared.data[idx] = self._merge_cell(
                            shared.data[idx], value, threshold
                        )
        return shared

    def _map_bounds(self, grid, is_r2, transform):
        corners = [(0, 0), (grid.info.width, 0), (0, grid.info.height),
                   (grid.info.width, grid.info.height)]
        points = []
        for ix, iy in corners:
            x = grid.info.origin.position.x + ix * grid.info.resolution
            y = grid.info.origin.position.y + iy * grid.info.resolution
            if is_r2:
                if transform is None:
                    return None
                x, y = self._transform_point(x, y, transform)
            points.append((x, y))
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        return min(xs), min(ys), max(xs), max(ys)

    def _grid_to_world(self, grid, ix, iy):
        return (
            grid.info.origin.position.x + (ix + 0.5) * grid.info.resolution,
            grid.info.origin.position.y + (iy + 0.5) * grid.info.resolution,
        )

    def _merge_cell(self, current, incoming, threshold):
        if incoming < 0:
            return current
        if current < 0:
            return incoming
        if incoming >= threshold or current >= threshold:
            return 100
        return 0

    def _publish_all(self):
        fusion_ok = self._fusion_allowed()
        accepted_tf = self._transform(use_candidate=False)

        raw = self._merge_maps(fusion_ok, accepted_tf)
        if raw is None and self.last_accepted_grid is not None:
            raw = self.last_accepted_grid
        if raw is None:
            raw = self._merge_maps(False, None)
        if raw is None:
            return

        if fusion_ok:
            self.last_accepted_grid = raw

        self.raw_pub.publish(raw)

        threshold = int(self.get_parameter("occupied_threshold").value)
        arr = np.asarray(raw.data, dtype=np.int16).reshape(
            raw.info.height, raw.info.width
        )
        cleaned_arr = clean_occupancy_grid(arr, occupied_threshold=threshold)
        cleaned = OccupancyGrid()
        cleaned.header = raw.header
        cleaned.info = raw.info
        cleaned.data = cleaned_arr.ravel().tolist()
        self.cleaned_pub.publish(cleaned)

        output = cleaned if bool(self.get_parameter("use_cleaned_shared_map").value) else raw
        self.shared_pub.publish(output)

        if self.candidate_valid and self._alignment_mode() in ("tag", "map", "hybrid"):
            cand_tf = self._transform(use_candidate=True)
            candidate_map = self._merge_maps(True, cand_tf)
            if candidate_map is not None:
                self.candidate_pub.publish(candidate_map)


def main(args=None):
    rclpy.init(args=args)
    node = SharedMapMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
