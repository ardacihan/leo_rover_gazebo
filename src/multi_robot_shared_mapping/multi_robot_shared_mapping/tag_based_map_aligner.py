#!/usr/bin/env python3
"""
Estimate leo2/map -> leo1/map from PERSISTENT AprilTag landmarks.

Each robot has a LandmarkMap in its own map frame. A tag seen once stays in
the map forever and is refined on re-observation (weighted average + outlier
gating), so leo1 can see tag_2 now and leo2 can see the same tag much later.

Publishes:
- /estimated_transform/leo2_to_leo1 (TransformStamped) + TF leo1/map -> leo2/map_estimated
- /tag_alignment_confidence (std_msgs/Float32)
- /leo1/apriltag_landmarks, /leo2/apriltag_landmarks (MarkerArray: cubes + text)
- /leo1/apriltag_landmarks_data, /leo2/apriltag_landmarks_data (String, JSON)
  consumed by save_shared_outputs.

Ground truth is only used for optional evaluation logs, never for alignment.
"""

from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from multi_robot_shared_mapping.tag_landmark_map import (
    LandmarkMap,
    compute_tag_alignment_confidence,
    geometric_spread,
    merge_landmark_dicts,
)
from multi_robot_shared_mapping.tag_map_alignment import (
    TransformEstimate,
    apply_2d_transform,
    estimate_2d_transform,
    estimate_transform_from_single_tag,
)

ROBOT_COLORS = {"leo1": (0.1, 0.9, 0.2), "leo2": (0.9, 0.5, 0.1)}
SHARED_COLOR = (0.2, 0.4, 1.0)


class TagBasedMapAligner(Node):
    def __init__(self):
        super().__init__("tag_based_map_aligner")

        self.declare_parameter("leo1_detections_topic", "/leo1/tag_detections")
        self.declare_parameter("leo2_detections_topic", "/leo2/tag_detections")
        self.declare_parameter("output_topic", "/estimated_transform/leo2_to_leo1")
        self.declare_parameter("confidence_topic", "/tag_alignment_confidence")
        self.declare_parameter("publish_estimated_tf", True)
        self.declare_parameter("parent_map_frame", "leo1/map")
        self.declare_parameter("child_map_frame", "leo2/map_estimated")

        # Persistent landmark behavior.
        self.declare_parameter("landmark_persistence", True)
        # Only used when landmark_persistence is false (debug fallback).
        self.declare_parameter("tag_cache_timeout_sec", 30.0)
        self.declare_parameter("landmark_gate_distance", 1.0)
        self.declare_parameter("landmark_gate_min_observations", 5)
        self.declare_parameter("landmark_publish_period_sec", 2.0)
        self.declare_parameter("status_log_period_sec", 5.0)

        self.declare_parameter("ground_truth_x", 2.36)
        self.declare_parameter("ground_truth_y", -11.27)
        self.declare_parameter("ground_truth_yaw", 0.0)
        self.declare_parameter("compare_to_ground_truth", False)
        self.declare_parameter("min_tags", 2)
        self.declare_parameter("max_mean_error", 0.35)

        # Robustness gating: tags are only an initial guess, so bad estimates
        # must be rejected or published with clearly low confidence.
        self.declare_parameter("min_common_landmarks_for_high_confidence", 3)
        self.declare_parameter("min_landmark_spread", 1.5)
        self.declare_parameter("max_tag_residual_mean", 0.75)
        self.declare_parameter("max_tag_residual_max", 1.5)

        # One common tag = weak relocalization hint only (never a full merge).
        self.declare_parameter("allow_single_tag_relocalization", True)
        self.declare_parameter("single_tag_max_confidence", 0.25)

        # Shared/global landmark map (leo1/map frame): leo2 landmarks are only
        # included while the accepted map transform confidence is high.
        self.declare_parameter("accepted_transform_topic", "/map_based_transform/leo2_to_leo1")
        self.declare_parameter("accepted_confidence_topic", "/alignment_confidence")
        self.declare_parameter("min_confidence_for_shared_landmarks", 0.5)

        gate = float(self.get_parameter("landmark_gate_distance").value)
        gate_min_obs = int(self.get_parameter("landmark_gate_min_observations").value)
        self.landmark_maps = {
            "leo1": LandmarkMap(gate_distance=gate, gate_min_observations=gate_min_obs),
            "leo2": LandmarkMap(gate_distance=gate, gate_min_observations=gate_min_obs),
        }
        self.last_transform: Optional[Tuple[float, float, float]] = None
        self.accepted_map_transform: Optional[Tuple[float, float, float]] = None
        self.accepted_map_confidence: Optional[float] = None
        self._last_status_log_sec = 0.0
        self._single_tag_logged = False

        leo1_topic = self.get_parameter("leo1_detections_topic").value
        leo2_topic = self.get_parameter("leo2_detections_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.create_subscription(
            MarkerArray, leo1_topic, lambda msg: self._detections_cb("leo1", msg), 10
        )
        self.create_subscription(
            MarkerArray, leo2_topic, lambda msg: self._detections_cb("leo2", msg), 10
        )
        self.transform_pub = self.create_publisher(TransformStamped, output_topic, 10)
        self.confidence_pub = self.create_publisher(
            Float32, str(self.get_parameter("confidence_topic").value), 10
        )
        self.landmark_pubs = {
            robot: self.create_publisher(MarkerArray, f"/{robot}/apriltag_landmarks", 10)
            for robot in ("leo1", "leo2")
        }
        self.landmark_data_pubs = {
            robot: self.create_publisher(String, f"/{robot}/apriltag_landmarks_data", 10)
            for robot in ("leo1", "leo2")
        }
        self.tag_debug_pub = self.create_publisher(String, "/tag_alignment_debug_json", 10)
        self.shared_landmarks_pub = self.create_publisher(
            MarkerArray, "/shared/apriltag_landmarks", 10
        )
        self.shared_landmarks_data_pub = self.create_publisher(
            String, "/shared/apriltag_landmarks_data", 10
        )
        self.create_subscription(
            TransformStamped,
            str(self.get_parameter("accepted_transform_topic").value),
            self._accepted_transform_cb, 10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("accepted_confidence_topic").value),
            self._accepted_confidence_cb, 10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.align_timer = self.create_timer(1.0, self.try_align)
        self.landmark_timer = self.create_timer(
            float(self.get_parameter("landmark_publish_period_sec").value),
            self._publish_landmarks,
        )

        persistence = bool(self.get_parameter("landmark_persistence").value)
        self.get_logger().info(
            f"Listening to {leo1_topic} and {leo2_topic}; publishing {output_topic} "
            f"(landmark_persistence={persistence})"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _detections_cb(self, robot: str, msg: MarkerArray):
        landmark_map = self.landmark_maps[robot]
        now = self._now_sec()
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            qz = marker.pose.orientation.z
            qw = marker.pose.orientation.w
            outcome = landmark_map.update(
                int(marker.id),
                marker.pose.position.x,
                marker.pose.position.y,
                2.0 * math.atan2(qz, qw),
                now,
            )
            if outcome == "rejected":
                self.get_logger().warn(
                    f"{robot}: rejected outlier detection of tag_{marker.id} "
                    "(too far from saved landmark)"
                )

        if not bool(self.get_parameter("landmark_persistence").value):
            self._prune_expired(landmark_map, now)

    def _prune_expired(self, landmark_map: LandmarkMap, now: float):
        """Debug fallback only: with persistence disabled, drop stale landmarks."""
        timeout = float(self.get_parameter("tag_cache_timeout_sec").value)
        stale = [
            tid for tid, lm in landmark_map.landmarks.items()
            if now - lm.last_seen_time > timeout
        ]
        for tid in stale:
            del landmark_map.landmarks[tid]

    def _accepted_transform_cb(self, msg: TransformStamped):
        yaw = 2.0 * math.atan2(msg.transform.rotation.z, msg.transform.rotation.w)
        self.accepted_map_transform = (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            yaw,
        )

    def _accepted_confidence_cb(self, msg: Float32):
        self.accepted_map_confidence = float(msg.data)

    def _make_landmark_markers(self, entries, frame_id, color, stamp) -> List[Marker]:
        """Cube + text markers for landmark dicts (tag_id, x, y, yaw)."""
        r, g, b = color
        markers = []
        for lm in entries:
            cube = Marker()
            cube.header.stamp = stamp
            cube.header.frame_id = frame_id
            cube.ns = "landmarks"
            cube.id = lm["tag_id"]
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = lm["x"]
            cube.pose.position.y = lm["y"]
            cube.pose.position.z = 0.25
            cube.pose.orientation.z = math.sin(lm["yaw"] / 2.0)
            cube.pose.orientation.w = math.cos(lm["yaw"] / 2.0)
            cube.scale.x = 0.2
            cube.scale.y = 0.2
            cube.scale.z = 0.5
            cube.color.r, cube.color.g, cube.color.b = r, g, b
            cube.color.a = 0.9

            text = Marker()
            text.header = cube.header
            text.ns = "labels"
            text.id = lm["tag_id"]
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = lm["x"]
            text.pose.position.y = lm["y"]
            text.pose.position.z = 0.75
            text.scale.z = 0.3
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"tag_{lm['tag_id']}"

            markers.extend([cube, text])
        return markers

    def _publish_landmarks(self):
        stamp = self.get_clock().now().to_msg()
        for robot, landmark_map in self.landmark_maps.items():
            entries = landmark_map.to_dict_list()
            markers = MarkerArray()
            markers.markers = self._make_landmark_markers(
                entries, f"{robot}/map", ROBOT_COLORS[robot], stamp
            )
            self.landmark_pubs[robot].publish(markers)
            self.landmark_data_pubs[robot].publish(String(data=json.dumps(entries)))
        self._publish_shared_landmarks(stamp)

    def _publish_shared_landmarks(self, stamp):
        """
        Global landmark map in leo1/map: leo1 landmarks directly, leo2
        landmarks transformed by the ACCEPTED map transform, and only while
        the accepted confidence is high. These global landmarks are the
        anchor for later relocalization (a robot re-observing one of them can
        propose a correction candidate, validated by map overlap as usual).
        """
        leo1_entries = self.landmark_maps["leo1"].to_dict_list()
        leo2_entries = []
        min_conf = float(self.get_parameter("min_confidence_for_shared_landmarks").value)
        if (
            self.accepted_map_transform is not None
            and self.accepted_map_confidence is not None
            and self.accepted_map_confidence >= min_conf
        ):
            leo2_entries = self.landmark_maps["leo2"].to_dict_list()

        merged = merge_landmark_dicts(
            leo1_entries, leo2_entries, self.accepted_map_transform or (0.0, 0.0, 0.0)
        )
        markers = MarkerArray()
        markers.markers = self._make_landmark_markers(
            merged, "leo1/map", SHARED_COLOR, stamp
        )
        self.shared_landmarks_pub.publish(markers)
        self.shared_landmarks_data_pub.publish(String(data=json.dumps(merged)))

    def _residuals(
        self, common_ids: List[int], estimate: TransformEstimate
    ) -> Tuple[float, float]:
        """Mean and max distance between transformed leo2 and saved leo1 landmarks."""
        errors = []
        for tid in common_ids:
            lm2 = self.landmark_maps["leo2"].landmarks[tid]
            lm1 = self.landmark_maps["leo1"].landmarks[tid]
            tx, ty = apply_2d_transform(lm2.x, lm2.y, estimate.dx, estimate.dy, estimate.yaw)
            errors.append(math.hypot(tx - lm1.x, ty - lm1.y))
        return sum(errors) / len(errors), max(errors)

    def _log_status(self, now: float, common_ids: List[int], reason: str):
        period = float(self.get_parameter("status_log_period_sec").value)
        if now - self._last_status_log_sec < period:
            return
        self._last_status_log_sec = now
        leo1 = self.landmark_maps["leo1"].landmarks
        leo2 = self.landmark_maps["leo2"].landmarks
        self.get_logger().info(
            f"landmarks leo1={len(leo1)} {sorted(leo1)} "
            f"leo2={len(leo2)} {sorted(leo2)} "
            f"common={common_ids} | {reason}"
        )

    def try_align(self):
        now = self._now_sec()
        map1 = self.landmark_maps["leo1"]
        map2 = self.landmark_maps["leo2"]
        common_ids = sorted(set(map1.landmarks) & set(map2.landmarks))
        min_tags = int(self.get_parameter("min_tags").value)
        allow_single = bool(self.get_parameter("allow_single_tag_relocalization").value)

        if len(common_ids) == 0:
            self.confidence_pub.publish(Float32(data=0.0))
            self._log_status(now, common_ids, "No common persistent landmarks.")
            return

        if len(common_ids) == 1 and not allow_single:
            self.confidence_pub.publish(Float32(data=0.0))
            self._log_status(
                now, common_ids,
                "Need at least 2 common persistent landmarks for tag-based alignment.",
            )
            return

        ground_truth = None
        if self.get_parameter("compare_to_ground_truth").value:
            ground_truth = (
                float(self.get_parameter("ground_truth_x").value),
                float(self.get_parameter("ground_truth_y").value),
                float(self.get_parameter("ground_truth_yaw").value),
            )

        single_tag = len(common_ids) == 1
        if single_tag:
            tid = common_ids[0]
            lm1 = map1.landmarks[tid]
            lm2 = map2.landmarks[tid]
            dx, dy, yaw = estimate_transform_from_single_tag(
                lm1.x, lm1.y, lm1.yaw, lm2.x, lm2.y, lm2.yaw
            )
            if not self._single_tag_logged:
                self._single_tag_logged = True
                self.get_logger().warn(
                    "Only one common landmark: weak/medium anchor — robots may "
                    "continue separate exploration; map overlap validates alignment."
                )
            estimate = TransformEstimate(
                dx=dx, dy=dy, yaw=yaw, success=True, confidence=0.2,
                mean_reprojection_error=0.0, num_tags=1,
                message="Single-tag weak relocalization hint",
            )
        else:
            estimate = estimate_2d_transform(
                [(map2.landmarks[tid].x, map2.landmarks[tid].y) for tid in common_ids],
                [(map1.landmarks[tid].x, map1.landmarks[tid].y) for tid in common_ids],
                min_tags=min_tags,
                max_mean_error=float(self.get_parameter("max_mean_error").value),
                ground_truth=ground_truth,
            )

        mean_residual, max_residual = self._residuals(common_ids, estimate)
        confidence = compute_tag_alignment_confidence(
            common_ids, map1, map2,
            mean_residual, max_residual,
            self.last_transform,
            (estimate.dx, estimate.dy, estimate.yaw),
        )
        confidence = self._apply_geometry_penalties(common_ids, confidence)
        if single_tag:
            confidence = min(
                confidence,
                float(self.get_parameter("single_tag_max_confidence").value),
            )

        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.tag_debug_pub.publish(String(data=json.dumps({
            "common_landmarks": common_ids,
            "tag_residual_mean": mean_residual,
            "tag_residual_max": max_residual,
            "landmark_spread": geometric_spread([
                (map1.landmarks[tid].x, map1.landmarks[tid].y) for tid in common_ids
            ]),
            "single_tag_hint": single_tag,
            "confidence": confidence,
        })))
        self._log_estimate(estimate, common_ids, mean_residual, max_residual, confidence)

        if not estimate.success:
            self._log_status(now, common_ids, f"transform rejected: {estimate.message}")
            return

        if not single_tag:
            max_res_mean = float(self.get_parameter("max_tag_residual_mean").value)
            max_res_max = float(self.get_parameter("max_tag_residual_max").value)
            if mean_residual > max_res_mean or max_residual > max_res_max:
                self._log_status(
                    now, common_ids,
                    f"transform rejected: residual too high "
                    f"(mean={mean_residual:.2f}>{max_res_mean} "
                    f"or max={max_residual:.2f}>{max_res_max})",
                )
                return

        self._log_status(now, common_ids, "transform published")
        self.last_transform = (estimate.dx, estimate.dy, estimate.yaw)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("parent_map_frame").value)
        transform.child_frame_id = str(self.get_parameter("child_map_frame").value)
        transform.transform.translation.x = estimate.dx
        transform.transform.translation.y = estimate.dy
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(estimate.yaw / 2.0)
        transform.transform.rotation.w = math.cos(estimate.yaw / 2.0)
        self.transform_pub.publish(transform)

        if self.get_parameter("publish_estimated_tf").value:
            self.tf_broadcaster.sendTransform(transform)

    def _apply_geometry_penalties(self, common_ids: List[int], confidence: float) -> float:
        """Cap confidence when tag count or geometric spread is too weak."""
        min_high_conf = int(
            self.get_parameter("min_common_landmarks_for_high_confidence").value
        )
        if len(common_ids) < min_high_conf:
            self.get_logger().warn(
                "Only 2 common landmarks; yaw estimate may be unstable."
            )
            confidence = min(confidence, 0.35)

        spread = geometric_spread([
            (self.landmark_maps["leo1"].landmarks[tid].x,
             self.landmark_maps["leo1"].landmarks[tid].y)
            for tid in common_ids
        ])
        if spread < float(self.get_parameter("min_landmark_spread").value):
            self.get_logger().warn("Common landmarks have poor geometric spread.")
            confidence *= 0.5
        return confidence

    def _log_estimate(
        self,
        estimate: TransformEstimate,
        tag_ids: List[int],
        mean_residual: float,
        max_residual: float,
        confidence: float,
    ):
        parts = [
            f"tags={tag_ids}",
            f"dx={estimate.dx:.3f}",
            f"dy={estimate.dy:.3f}",
            f"yaw={estimate.yaw:.3f}",
            f"residual mean={mean_residual:.3f} max={max_residual:.3f}",
            f"conf={confidence:.2f}",
            estimate.message,
        ]
        if estimate.translation_error is not None:
            parts.append(f"gt_trans_err={estimate.translation_error:.3f}")
        if estimate.yaw_error is not None:
            parts.append(f"gt_yaw_err={math.degrees(estimate.yaw_error):.1f}deg")

        text = " | ".join(parts)
        if estimate.success:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = TagBasedMapAligner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
