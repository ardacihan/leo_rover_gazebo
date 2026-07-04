#!/usr/bin/env python3
"""
Consistency-aware alignment manager for map / hybrid / tag modes.

Computes occupancy-grid match candidates (with top-K ambiguity detection),
combines tag + map evidence into final_alignment_confidence, and maintains
an accepted transform separate from every new candidate.

Publishes (non-fixed modes):
- /map_based_transform/leo2_to_leo1  accepted transform only
- /alignment_candidate_transform      latest candidate (always when computed)
- /alignment_confidence               final confidence of the candidate
- /alignment_debug_json               full diagnostic JSON
- /alignment_recovery_goal              recovery recommendation JSON when low
- /leo2/map_transformed_debug         best map-match candidate in leo1/map

Ground truth is never used for alignment decisions.
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32, String

from multi_robot_shared_mapping.alignment_confidence import (
    ConfidenceInputs,
    compute_final_confidence,
    detect_ambiguity,
    select_top_candidates,
    transforms_disagree,
)
from multi_robot_shared_mapping.alignment_state import AlignmentState
from multi_robot_shared_mapping.grid_map_matching import (
    GridMatchResult,
    downsample_points,
    match_maps,
    occupancy_grid_to_points,
)
from multi_robot_shared_mapping.map_quality import LocalMapQualityTracker
from multi_robot_shared_mapping.exploration_policy import (
    build_policy_debug,
    min_acceptance_confidence,
    tag_map_agree,
)
from multi_robot_shared_mapping.recovery_advisor import recommend_recovery
from multi_robot_shared_mapping.tag_landmark_map import geometric_spread


class MapBasedAligner(Node):
    def __init__(self):
        super().__init__("map_based_aligner")

        self.declare_parameter("alignment_mode", "hybrid")
        self.declare_parameter("map1_topic", "/leo1/map")
        self.declare_parameter("map2_topic", "/leo2/map")
        self.declare_parameter("tag_transform_topic", "/estimated_transform/leo2_to_leo1")
        self.declare_parameter("tag_confidence_topic", "/tag_alignment_confidence")
        self.declare_parameter("tag_debug_topic", "/tag_alignment_debug_json")
        self.declare_parameter("leo1_landmarks_data_topic", "/leo1/apriltag_landmarks_data")
        self.declare_parameter("leo2_landmarks_data_topic", "/leo2/apriltag_landmarks_data")
        self.declare_parameter("output_topic", "/map_based_transform/leo2_to_leo1")
        self.declare_parameter("candidate_topic", "/alignment_candidate_transform")
        self.declare_parameter("confidence_topic", "/alignment_confidence")
        self.declare_parameter("debug_topic", "/alignment_debug_json")
        self.declare_parameter("recovery_topic", "/alignment_recovery_goal")
        self.declare_parameter("parent_map_frame", "leo1/map")
        self.declare_parameter("child_map_frame", "leo2/map_grid_estimated")
        self.declare_parameter("match_period_sec", 5.0)

        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("match_resolution", 0.15)
        self.declare_parameter("max_match_points", 400)
        self.declare_parameter("map_search_range_xy", 15.0)
        self.declare_parameter("map_search_range_yaw", math.pi)
        self.declare_parameter("hybrid_search_range_xy", 2.0)
        self.declare_parameter("hybrid_search_range_yaw", 0.35)

        self.declare_parameter("min_occupied_cells", 100)
        self.declare_parameter("min_overlap_score", 30)
        self.declare_parameter("min_alignment_confidence", 0.5)
        self.declare_parameter("min_confidence_improvement", 0.05)
        self.declare_parameter("map_mode_min_confidence", 0.6)
        self.declare_parameter("max_transform_jump", 2.0)
        self.declare_parameter("max_yaw_jump_deg", 25.0)
        self.declare_parameter("require_consistency_for_update", True)

        self.declare_parameter("max_tag_map_disagreement_m", 1.0)
        self.declare_parameter("max_tag_map_disagreement_yaw_deg", 15.0)
        self.declare_parameter("max_free_space_conflict_ratio", 0.15)
        self.declare_parameter("min_occupied_overlap_ratio", 0.25)
        self.declare_parameter("reset_bad_local_map_recommended", False)
        self.declare_parameter("debug_map_topic", "/leo2/map_transformed_debug")

        self.map1: Optional[OccupancyGrid] = None
        self.map2: Optional[OccupancyGrid] = None
        self.tag_estimate: Optional[Tuple[float, float, float]] = None
        self.tag_confidence: Optional[float] = None
        self.tag_residual_mean: Optional[float] = None
        self.tag_residual_max: Optional[float] = None
        self.common_landmarks: List[int] = []
        self.landmark_spread: float = 0.0
        self.leo1_landmarks: Dict[int, Tuple[float, float]] = {}
        self.leo2_landmarks: Dict[int, Tuple[float, float]] = {}

        self.state = AlignmentState(
            min_alignment_confidence=float(self.get_parameter("min_alignment_confidence").value),
            min_confidence_improvement=float(self.get_parameter("min_confidence_improvement").value),
            max_transform_jump=float(self.get_parameter("max_transform_jump").value),
            max_yaw_jump=math.radians(float(self.get_parameter("max_yaw_jump_deg").value)),
            require_consistency_for_update=bool(
                self.get_parameter("require_consistency_for_update").value
            ),
        )
        self.quality_leo1 = LocalMapQualityTracker()
        self.quality_leo2 = LocalMapQualityTracker()
        self._idle_logged = False
        self._last_recovery: Optional[str] = None

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map1_topic").value), self._map1_cb, 10
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map2_topic").value), self._map2_cb, 10
        )
        self.create_subscription(
            TransformStamped, str(self.get_parameter("tag_transform_topic").value),
            self._tag_transform_cb, 10,
        )
        self.create_subscription(
            Float32, str(self.get_parameter("tag_confidence_topic").value),
            self._tag_confidence_cb, 10,
        )
        self.create_subscription(
            String, str(self.get_parameter("tag_debug_topic").value),
            self._tag_debug_cb, 10,
        )
        self.create_subscription(
            String, str(self.get_parameter("leo1_landmarks_data_topic").value),
            lambda m: self._landmarks_cb(m, "leo1"), 10,
        )
        self.create_subscription(
            String, str(self.get_parameter("leo2_landmarks_data_topic").value),
            lambda m: self._landmarks_cb(m, "leo2"), 10,
        )

        self.accepted_pub = self.create_publisher(
            TransformStamped, str(self.get_parameter("output_topic").value), 10
        )
        self.candidate_pub = self.create_publisher(
            TransformStamped, str(self.get_parameter("candidate_topic").value), 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, str(self.get_parameter("confidence_topic").value), 10
        )
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10
        )
        self.recovery_pub = self.create_publisher(
            String, str(self.get_parameter("recovery_topic").value), 10
        )
        self.debug_map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("debug_map_topic").value), 10
        )
        self.timer = self.create_timer(
            float(self.get_parameter("match_period_sec").value), self._cycle
        )

        self.get_logger().info(
            f"alignment manager started (mode={self._mode()}); accepted -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _mode(self) -> str:
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

    def _tag_transform_cb(self, msg: TransformStamped):
        yaw = 2.0 * math.atan2(msg.transform.rotation.z, msg.transform.rotation.w)
        self.tag_estimate = (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            yaw,
        )

    def _tag_confidence_cb(self, msg: Float32):
        self.tag_confidence = float(msg.data)

    def _tag_debug_cb(self, msg: String):
        data = json.loads(msg.data)
        self.tag_residual_mean = data.get("tag_residual_mean")
        self.tag_residual_max = data.get("tag_residual_max")
        if "landmark_spread" in data and data.get("common_landmarks"):
            self.landmark_spread = float(data["landmark_spread"])
            self.common_landmarks = list(data.get("common_landmarks", []))

    def _landmarks_cb(self, msg: String, robot: str):
        entries = json.loads(msg.data)
        store = self.leo1_landmarks if robot == "leo1" else self.leo2_landmarks
        store.clear()
        for lm in entries:
            store[int(lm["tag_id"])] = (float(lm["x"]), float(lm["y"]))
        self.common_landmarks = sorted(set(self.leo1_landmarks) & set(self.leo2_landmarks))
        if self.common_landmarks:
            self.landmark_spread = geometric_spread([
                self.leo1_landmarks[tid] for tid in self.common_landmarks
            ])

    def _grid_points(self, grid: OccupancyGrid, select: str = "occupied"):
        threshold = int(self.get_parameter("occupied_threshold").value)
        return occupancy_grid_to_points(
            grid.data, grid.info.width, grid.info.height,
            grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y,
            occupied_threshold=threshold, select=select,
        )

    def _cycle(self):
        mode = self._mode()
        if mode == "fixed":
            if not self._idle_logged:
                self._idle_logged = True
                self.get_logger().info("alignment_mode=fixed: alignment manager idle")
            return

        if mode == "tag":
            self._cycle_tag_or_map(fallback_tag_only=True)
            return

        self._cycle_tag_or_map(fallback_tag_only=False)

    def _cycle_tag_or_map(self, fallback_tag_only: bool):
        """Map matching always attempted when maps exist; tags refine when present."""
        if self.map1 is not None and self.map2 is not None:
            self._cycle_map_or_hybrid(fallback_tag_only=fallback_tag_only)
            return
        if fallback_tag_only:
            self._cycle_tag_only()

    def _cycle_tag_only(self):
        """Tag hint only when maps are not yet available."""
        if self.tag_estimate is None:
            return
        candidate = self.tag_estimate
        self._publish_transform(self.candidate_pub, candidate)

        local_q = min(self.quality_leo1.quality, self.quality_leo2.quality)
        conf_inputs = ConfidenceInputs(
            occupancy_overlap_score=0.0,
            free_space_conflict_score=1.0,
            transform_stability_score=self._stability(candidate),
            unambiguity_score=1.0,
            local_map_quality_score=local_q,
            tag_alignment_confidence=self.tag_confidence,
            tag_residual_score=self._tag_residual_score(),
            common_landmark_count_score=min(1.0, len(self.common_landmarks) / 4.0),
            landmark_spread_score=min(1.0, self.landmark_spread / 2.0),
        )
        confidence = compute_final_confidence(conf_inputs)
        self._finalize(candidate, confidence, conf_inputs, None, False, "")

    def _cycle_map_or_hybrid(self, fallback_tag_only: bool = False):
        if self.map1 is None or self.map2 is None:
            return

        if self.quality_leo1.is_poor or self.quality_leo2.is_poor:
            for name, tracker in (("leo1", self.quality_leo1), ("leo2", self.quality_leo2)):
                if tracker.is_poor:
                    self.get_logger().warn(
                        f"Local map quality low for {name}; holding fusion and "
                        "requesting relocalization."
                    )

        target_points = self._grid_points(self.map1)
        source_points = self._grid_points(self.map2)
        min_cells = int(self.get_parameter("min_occupied_cells").value)
        if len(target_points) < min_cells or len(source_points) < min_cells:
            return

        match_resolution = float(self.get_parameter("match_resolution").value)
        max_points = int(self.get_parameter("max_match_points").value)
        target_down = downsample_points(target_points, match_resolution, max_points)
        source_down = downsample_points(source_points, match_resolution, max_points)
        free_down = downsample_points(
            self._grid_points(self.map1, select="free"), match_resolution, max_points * 4
        )

        window = self._search_window()
        if window is None:
            return
        center, xy_range, yaw_range = window

        result = match_maps(
            source_down, target_down,
            match_resolution=match_resolution,
            center=center, xy_range=xy_range, yaw_range=yaw_range,
            target_free_points=free_down,
        )
        self._publish_debug_map(result)

        top_k = select_top_candidates(result.candidates, k=5)
        ambiguity_ratio, is_ambiguous = detect_ambiguity(top_k)

        # Hybrid/tag/map: occupancy overlap always computed when maps exist.
        extra_reject = self._preflight_reject(result, is_ambiguous)

        local_q = min(self.quality_leo1.quality, self.quality_leo2.quality)
        free_conflict_score = max(
            0.0, 1.0 - result.free_space_conflict_ratio
            / max(1e-6, float(self.get_parameter("max_free_space_conflict_ratio").value))
        )
        conf_inputs = ConfidenceInputs(
            occupancy_overlap_score=result.normalized_overlap_score,
            free_space_conflict_score=free_conflict_score,
            transform_stability_score=self._stability((result.dx, result.dy, result.yaw)),
            unambiguity_score=max(0.0, 1.0 - ambiguity_ratio),
            local_map_quality_score=local_q,
            tag_alignment_confidence=self.tag_confidence if self._mode() in ("hybrid", "tag") else None,
            tag_residual_score=self._tag_residual_score(),
            common_landmark_count_score=min(1.0, len(self.common_landmarks) / 4.0),
            landmark_spread_score=min(1.0, self.landmark_spread / 2.0),
        )
        confidence = compute_final_confidence(conf_inputs)
        candidate = (result.dx, result.dy, result.yaw)
        self._finalize(
            candidate, confidence, conf_inputs, result, is_ambiguous,
            extra_reject or "", top_k=top_k, ambiguity_ratio=ambiguity_ratio,
        )

    def _preflight_reject(self, result: GridMatchResult, is_ambiguous: bool) -> str:
        mode = self._mode()
        if not result.success:
            return result.message
        if result.normalized_overlap_score < float(
            self.get_parameter("min_occupied_overlap_ratio").value
        ):
            return (
                f"overlap ratio {result.normalized_overlap_score:.2f} < "
                f"min_occupied_overlap_ratio"
            )
        if result.free_space_conflict_ratio > float(
            self.get_parameter("max_free_space_conflict_ratio").value
        ):
            return (
                f"free-space conflict {result.free_space_conflict_ratio:.2f} > "
                f"max_free_space_conflict_ratio"
            )
        # Ambiguous map-only: candidate only, never accepted.
        if mode in ("map", "hybrid", "tag") and is_ambiguous and len(self.common_landmarks) == 0:
            return "ambiguous map-only match; candidate only until geometry is distinctive"
        if (
            mode in ("hybrid", "tag")
            and len(self.common_landmarks) == 1
            and self.tag_estimate is not None
            and transforms_disagree(
                self.tag_estimate,
                (result.dx, result.dy, result.yaw),
                float(self.get_parameter("max_tag_map_disagreement_m").value),
                math.radians(float(self.get_parameter("max_tag_map_disagreement_yaw_deg").value)),
            )
        ):
            return "Rejected candidate: tag alignment and occupancy alignment disagree"
        if self.quality_leo1.is_severe or self.quality_leo2.is_severe:
            return "local map quality too low for fusion"
        return ""

    def _finalize(
        self,
        candidate: Tuple[float, float, float],
        confidence: float,
        conf_inputs: ConfidenceInputs,
        result: Optional[GridMatchResult],
        is_ambiguous: bool,
        extra_reject: str,
        top_k: Optional[list] = None,
        ambiguity_ratio: float = 0.0,
    ):
        mode = self._mode()
        self._publish_transform(self.candidate_pub, candidate)
        self.confidence_pub.publish(Float32(data=float(confidence)))

        overlap = conf_inputs.occupancy_overlap_score
        agreement = tag_map_agree(
            self.tag_estimate,
            candidate,
            float(self.get_parameter("max_tag_map_disagreement_m").value),
            math.radians(float(self.get_parameter("max_tag_map_disagreement_yaw_deg").value)),
        )
        base_min = float(self.get_parameter("min_alignment_confidence").value)
        map_min = float(self.get_parameter("map_mode_min_confidence").value)
        min_conf = min_acceptance_confidence(
            len(self.common_landmarks),
            mode,
            overlap,
            is_ambiguous=is_ambiguous,
            tag_map_agreement=agreement,
            base_min=base_min,
            map_mode_min=map_min,
        )
        self.state.min_alignment_confidence = min_conf

        policy_preview = build_policy_debug(
            mode=mode,
            final_confidence=confidence,
            common_landmark_count=len(self.common_landmarks),
            map_overlap_score=overlap,
            ambiguity_score=ambiguity_ratio,
            is_ambiguous=is_ambiguous,
            tag_map_agreement=agreement,
            leo1_quality=self.quality_leo1.quality,
            leo2_quality=self.quality_leo2.quality,
            recovery=None,
            base_min=base_min,
            map_mode_min=map_min,
        )
        recovery = recommend_recovery(
            confidence=confidence,
            min_confidence=min_conf,
            common_ids=self.common_landmarks,
            leo1_landmarks=self.leo1_landmarks,
            leo2_landmarks=self.leo2_landmarks,
            is_ambiguous=is_ambiguous,
            leo1_quality=self.quality_leo1.quality,
            leo2_quality=self.quality_leo2.quality,
            reset_bad_local_map_recommended=bool(
                self.get_parameter("reset_bad_local_map_recommended").value
            ),
            exploration_allowed=policy_preview["exploration_allowed"],
            map_overlap_score=overlap,
        )
        policy = build_policy_debug(
            mode=mode,
            final_confidence=confidence,
            common_landmark_count=len(self.common_landmarks),
            map_overlap_score=overlap,
            ambiguity_score=ambiguity_ratio,
            is_ambiguous=is_ambiguous,
            tag_map_agreement=agreement,
            leo1_quality=self.quality_leo1.quality,
            leo2_quality=self.quality_leo2.quality,
            recovery=recovery,
            base_min=base_min,
            map_mode_min=map_min,
        )

        if extra_reject:
            accepted, reason = False, extra_reject
            self.state.evaluate_candidate(candidate, confidence, extra_reject_reason=extra_reject)
        else:
            accepted, reason = self.state.evaluate_candidate(candidate, confidence)

        debug = self.state.debug_dict(
            mode,
            occupancy_overlap_score=conf_inputs.occupancy_overlap_score,
            free_space_conflict_score=conf_inputs.free_space_conflict_score,
            tag_alignment_confidence=conf_inputs.tag_alignment_confidence,
            tag_residual_mean=self.tag_residual_mean,
            tag_residual_max=self.tag_residual_max,
            common_landmarks=self.common_landmarks,
            landmark_spread=self.landmark_spread,
            ambiguity_score=ambiguity_ratio,
            local_map_quality_leo1=self.quality_leo1.quality,
            local_map_quality_leo2=self.quality_leo2.quality,
            final_confidence=confidence,
            accepted=accepted,
            reason=reason,
            top_candidates=[
                {"dx": c[0], "dy": c[1], "yaw": c[2], "score": c[3]}
                for c in (top_k or [])
            ],
        )
        debug.update(policy)
        self.debug_pub.publish(String(data=json.dumps(debug)))

        tag_initial = (
            f"tag_initial=({self.tag_estimate[0]:.2f},{self.tag_estimate[1]:.2f},"
            f"{self.tag_estimate[2]:.2f})"
            if self.tag_estimate else "tag_initial=none"
        )
        overlap_text = (
            f"overlap={result.overlap_score} norm={result.normalized_overlap_score:.2f}"
            if result else "overlap=n/a"
        )
        log = (
            f"mode={mode} {tag_initial} | "
            f"candidate=({candidate[0]:.2f},{candidate[1]:.2f},{candidate[2]:.2f}) "
            f"conf={confidence:.2f} level={policy['confidence_level']} "
            f"explore={policy['exploration_allowed']} | {overlap_text} | "
            f"{'ACCEPTED' if accepted else 'REJECTED'}: {reason}"
        )
        if accepted:
            self.get_logger().info(log)
            if self.state.accepted is not None:
                self._publish_transform(self.accepted_pub, self.state.accepted)
        else:
            self.get_logger().warn(log)

        if recovery:
            payload = json.dumps(recovery)
            if payload != self._last_recovery:
                self._last_recovery = payload
                self.recovery_pub.publish(String(data=payload))
        elif self._last_recovery is not None:
            self._last_recovery = None

    def _search_window(self):
        mode = self._mode()
        if mode in ("hybrid", "tag") and self.tag_estimate is not None:
            return (
                self.tag_estimate,
                float(self.get_parameter("hybrid_search_range_xy").value),
                float(self.get_parameter("hybrid_search_range_yaw").value),
            )
        if self.state.accepted is not None:
            return (
                self.state.accepted,
                float(self.get_parameter("hybrid_search_range_xy").value),
                float(self.get_parameter("hybrid_search_range_yaw").value),
            )
        # No tags yet: full map search for collaborative map-only alignment.
        return (
            (0.0, 0.0, 0.0),
            float(self.get_parameter("map_search_range_xy").value),
            float(self.get_parameter("map_search_range_yaw").value),
        )

    def _stability(self, candidate: Tuple[float, float, float]) -> float:
        ref = self.state.accepted
        if ref is None:
            return 0.5
        jump = math.hypot(candidate[0] - ref[0], candidate[1] - ref[1])
        return 1.0 / (1.0 + jump)

    def _tag_residual_score(self) -> Optional[float]:
        if self.tag_residual_mean is None:
            return None
        return 1.0 / (1.0 + self.tag_residual_mean)

    def _publish_transform(self, publisher, transform: Tuple[float, float, float]):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("parent_map_frame").value)
        msg.child_frame_id = str(self.get_parameter("child_map_frame").value)
        msg.transform.translation.x = transform[0]
        msg.transform.translation.y = transform[1]
        msg.transform.rotation.z = math.sin(transform[2] / 2.0)
        msg.transform.rotation.w = math.cos(transform[2] / 2.0)
        publisher.publish(msg)

    def _publish_debug_map(self, result: GridMatchResult):
        grid = self.map2
        if grid is None:
            return
        res = grid.info.resolution
        values = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width
        )
        iy, ix = np.nonzero(values >= 0)
        if len(ix) == 0:
            return
        known = values[iy, ix]
        xs = grid.info.origin.position.x + (ix + 0.5) * res
        ys = grid.info.origin.position.y + (iy + 0.5) * res
        c, s = math.cos(result.yaw), math.sin(result.yaw)
        tx = c * xs - s * ys + result.dx
        ty = s * xs + c * ys + result.dy

        min_x, min_y = float(tx.min()), float(ty.min())
        width = max(1, int(math.ceil((float(tx.max()) - min_x) / res)) + 1)
        height = max(1, int(math.ceil((float(ty.max()) - min_y) / res)) + 1)

        out = np.full((height, width), -1, dtype=np.int16)
        ox = np.floor((tx - min_x) / res).astype(int)
        oy = np.floor((ty - min_y) / res).astype(int)
        out[oy, ox] = known

        debug = OccupancyGrid()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = str(self.get_parameter("parent_map_frame").value)
        debug.info.resolution = res
        debug.info.width = width
        debug.info.height = height
        debug.info.origin.position.x = min_x
        debug.info.origin.position.y = min_y
        debug.info.origin.orientation.w = 1.0
        debug.data = out.ravel().tolist()
        self.debug_map_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = MapBasedAligner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
