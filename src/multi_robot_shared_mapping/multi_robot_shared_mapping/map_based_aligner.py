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
from std_msgs.msg import Bool, Float32, String

from multi_robot_shared_mapping.alignment_confidence import (
    ConfidenceInputs,
    compute_final_confidence,
    detect_ambiguity,
    select_top_candidates,
)
from multi_robot_shared_mapping.alignment_state import AlignmentState
from multi_robot_shared_mapping.grid_map_matching import (
    GridMatchResult,
    downsample_points,
    match_maps,
    occupancy_grid_to_points,
)
from multi_robot_shared_mapping.grid_registration import (
    info_from_message, local_refine, quaternion_yaw)
from multi_robot_shared_mapping import marker_free_matching
from multi_robot_shared_mapping.map_quality import LocalMapQualityTracker
from multi_robot_shared_mapping.exploration_policy import (
    build_policy_debug,
    fusion_evidence_rejection,
    min_acceptance_confidence,
    tag_map_agree,
)
from multi_robot_shared_mapping.recovery_advisor import recommend_recovery
from multi_robot_shared_mapping.tag_landmark_map import geometric_spread
from multi_robot_shared_mapping.geometric_residual import (
    geometric_lock_ok, residual_stats)


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
        self.declare_parameter("accepted_confidence_topic",
                               "/accepted_alignment_confidence")
        self.declare_parameter("debug_topic", "/alignment_debug_json")
        self.declare_parameter("recovery_topic", "/alignment_recovery_goal")
        self.declare_parameter("parent_map_frame", "leo1/map")
        self.declare_parameter("child_map_frame", "leo2/map_grid_estimated")
        self.declare_parameter("match_period_sec", 5.0)
        # markerfree mode: full global search costs 2-4 s, so run it only
        # every Nth cycle (N * match_period_sec between attempts).
        self.declare_parameter("markerfree_every_n", 3)

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
        self.declare_parameter("min_common_landmarks_for_hybrid", 2)
        self.declare_parameter("max_free_space_conflict_ratio", 0.15)
        self.declare_parameter("min_occupied_overlap_ratio", 0.25)
        self.declare_parameter("min_refined_wall_hit", 0.50)
        # The coarse grid search is quantised at 0.75 m / 15 deg, so a correct
        # polish routinely moves further than the old 0.90 m / 6 deg region -
        # it was the single largest reject bucket (132/293 in one run). The
        # geometry gate still judges the polished pose on wall residuals.
        self.declare_parameter("max_refinement_translation_m", 2.5)
        self.declare_parameter("max_refinement_yaw_deg", 12.0)
        # Confidence alone must not lock a still-shifted overlay.
        self.declare_parameter("max_lock_residual_m", 0.12)
        self.declare_parameter("max_lock_residual_p90_m", 0.30)
        self.declare_parameter("min_undilated_wall_hit", 0.30)
        # Before anything is accepted, both rovers must have mapped enough
        # that landmark positions are trustworthy. The heavy lifting against
        # grid aliasing (mirror locks at ~30 m2, +3 m shifts at ~80 m2) is
        # done by the common-landmark witness requirement and the landmark
        # conflict veto; this size gate only screens out the earliest maps.
        # It is raw grid area INCLUDING the exterior halo, and it must stay
        # small enough for compact worlds: at 120 the husarion_office leo2
        # map (85 m2 raw, 3 agreeing landmarks) was blocked for a whole run.
        self.declare_parameter("min_known_m2_for_first_lock", 60.0)
        # How far a shared landmark may sit from the peer's estimate of the
        # same landmark under a candidate transform. Registry noise is ~0.7 m
        # mean / 1.6 m max; wrong grid modes displace landmarks 12+ m.
        self.declare_parameter("max_landmark_conflict_m", 2.5)
        # After a lock, re-run the full global search every Nth cycle. The
        # tracking window is +/-2 m around the accepted pose, so without this
        # a wrong early lock is never challenged: the true mode sits outside
        # the window and the supersede path can never see a better candidate.
        self.declare_parameter("global_recheck_every_n", 4)
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
        self._mf_counter = 0
        self._cycle_count = 0
        self._global_recheck = False
        self._accepted_evidence = {}

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
        self.accepted_confidence_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("accepted_confidence_topic").value), 10)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10
        )
        self.recovery_pub = self.create_publisher(
            String, str(self.get_parameter("recovery_topic").value), 10
        )
        self.debug_map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("debug_map_topic").value), 10
        )
        self.residual_pub = self.create_publisher(
            Float32, "/alignment_residual_m", 10
        )
        self.geometry_ok_pub = self.create_publisher(
            Bool, "/alignment_geometry_ok", 10
        )
        self.accepted_validation_pub = self.create_publisher(
            String, "/accepted_alignment_validation", 10
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
            quaternion_yaw(grid.info.origin.orientation),
            occupied_threshold=threshold, select=select,
        )

    def _cycle(self):
        mode = self._mode()
        if mode == "fixed":
            if not self._idle_logged:
                self._idle_logged = True
                self.get_logger().info("alignment_mode=fixed: alignment manager idle")
            return

        if mode == "markerfree":
            self._cycle_markerfree()
            return

        if mode == "tag":
            self._cycle_tag_or_map(fallback_tag_only=True)
            return

        self._cycle_tag_or_map(fallback_tag_only=False)

    def _grid_tuple(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        info = info_from_message(msg)
        return grid, info

    def _cycle_markerfree(self):
        """Marker-free global merge: benchmarked 6/7 correct within 0.65 m,
        zero confident-wrong, on the 10 recorded map pairs. Abstains (and
        says why) rather than committing an ambiguous or low-overlap match;
        the tag pipeline is never consulted."""
        if self.map1 is None or self.map2 is None:
            return
        self._mf_counter += 1
        every = max(1, int(self.get_parameter("markerfree_every_n").value))
        if (self._mf_counter - 1) % every != 0:
            return

        g1, i1 = self._grid_tuple(self.map1)
        g2, i2 = self._grid_tuple(self.map2)
        min_cells = int(self.get_parameter("min_occupied_cells").value)
        if int((g1 >= 50).sum()) < min_cells or int((g2 >= 50).sum()) < min_cells:
            return

        est, diag = marker_free_matching.match_diag(g1, i1, g2, i2)
        if est is None:
            payload = {
                "mode": "markerfree", "abstained": True,
                "reason": diag.get("reason"),
                "best_hit": diag.get("best_hit"),
                "margin": diag.get("margin"),
                "n_modes": diag.get("n_modes"),
                "holding_accepted": self.state.accepted is not None,
            }
            self.debug_pub.publish(String(data=json.dumps(payload)))
            self.get_logger().info(f"markerfree: abstain ({diag.get('reason')})")
            if self.state.accepted is not None:
                # keep the last accepted transform alive downstream
                self._publish_transform(self.accepted_pub, self.state.accepted)
            return

        candidate = (float(est[0]), float(est[1]), float(est[2]))
        hit = float(diag["best_hit"])
        margin = diag.get("margin")
        conf_inputs = ConfidenceInputs(
            occupancy_overlap_score=hit,
            free_space_conflict_score=1.0,
            transform_stability_score=self._stability(candidate),
            unambiguity_score=(
                1.0 if margin is None else max(0.0, min(1.0, margin / 0.6))
            ),
            local_map_quality_score=min(
                self.quality_leo1.quality, self.quality_leo2.quality
            ),
            tag_alignment_confidence=None,
            tag_residual_score=None,
            common_landmark_count_score=0.0,
            landmark_spread_score=0.0,
        )
        # The polished wall-overlap fraction IS the calibrated confidence:
        # committed matches measured 0.87-0.998, flips 0.05-0.34.
        confidence = min(0.95, hit)
        self._finalize(candidate, confidence, conf_inputs, None, False, "")

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

        # The sampled matcher identifies a mode. A full-resolution local
        # polish removes the residual raster/SLAM offset before any lock is
        # allowed. Geometry remains the primary seed.  Markers may only pick
        # among modes that the grid matcher already found when those modes are
        # ambiguous; a noisy landmark fit must never pull refinement into an
        # unrelated transform basin.
        # Once a pose is accepted, refine that exact pose against the newest
        # map pair. A later noisy landmark fit must not drag the local polish
        # outside its trust region or make a good accepted merge go stale.
        # Tags remain an independent cross-check for the refined result.
        seed = (result.dx, result.dy, result.yaw)
        if self.state.accepted is not None and not self._global_recheck:
            seed = self.state.accepted
        elif (is_ambiguous and self._mode() in ("hybrid", "tag")
              and len(self.common_landmarks) >= 2
              and self.tag_estimate is not None):
            for mode_candidate in top_k:
                geometric_mode = mode_candidate[:3]
                if tag_map_agree(
                        self.tag_estimate, geometric_mode,
                        float(self.get_parameter(
                            "max_tag_map_disagreement_m").value),
                        math.radians(float(self.get_parameter(
                            "max_tag_map_disagreement_yaw_deg").value))):
                    seed = geometric_mode
                    break
        # One shared landmark is already enough to falsify a grid-aliasing
        # mode ~13 m from the truth. If the best-scoring mode contradicts a
        # common landmark, polish the best mode that does not.
        if (self.common_landmarks
                and (self.state.accepted is None or self._global_recheck)
                and self._landmark_conflict(seed)):
            for mode_candidate in top_k:
                if not self._landmark_conflict(mode_candidate[:3]):
                    seed = mode_candidate[:3]
                    break
        g1, i1 = self._grid_tuple(self.map1)
        g2, i2 = self._grid_tuple(self.map2)
        candidate, registration = local_refine(g1, i1, g2, i2, seed)

        # Rectilinear offices often have a second grid-only mode. Two or more
        # common landmarks that agree with the refined pose disambiguate that
        # mode; without them, a hybrid candidate remains preview-only however
        # attractive its occupancy residual looks.
        marker_disambiguated = (
            self._mode() in ("hybrid", "tag")
            and len(self.common_landmarks) >= 2
            and tag_map_agree(
                self.tag_estimate, candidate,
                float(self.get_parameter("max_tag_map_disagreement_m").value),
                math.radians(float(self.get_parameter(
                    "max_tag_map_disagreement_yaw_deg").value))))
        effective_ambiguous = is_ambiguous and not marker_disambiguated

        # Hybrid/tag/map: occupancy overlap always computed when maps exist.
        extra_reject = self._preflight_reject(
            result, effective_ambiguous, candidate, registration)

        local_q = min(self.quality_leo1.quality, self.quality_leo2.quality)
        free_conflict_score = max(
            0.0, 1.0 - result.free_space_conflict_ratio
            / max(1e-6, float(self.get_parameter("max_free_space_conflict_ratio").value))
        )
        conf_inputs = ConfidenceInputs(
            occupancy_overlap_score=registration['wall_hit'],
            free_space_conflict_score=free_conflict_score,
            transform_stability_score=self._stability(candidate),
            unambiguity_score=(1.0 if marker_disambiguated
                               else max(0.0, 1.0 - ambiguity_ratio)),
            local_map_quality_score=local_q,
            # A tag channel with nothing to say (no common markers, conf ~0)
            # must not sit in the weighted mix - its 0.10 weight suppressed
            # map-only confidence by ~0.09 for entire runs.
            tag_alignment_confidence=(
                self.tag_confidence
                if self._mode() in ("hybrid", "tag")
                and (self.tag_confidence or 0.0) > 0.05 else None),
            tag_residual_score=self._tag_residual_score(),
            common_landmark_count_score=min(1.0, len(self.common_landmarks) / 4.0),
            landmark_spread_score=min(1.0, self.landmark_spread / 2.0),
        )
        confidence = compute_final_confidence(conf_inputs)
        self._finalize(
            candidate, confidence, conf_inputs, result, effective_ambiguous,
            extra_reject or "", top_k=top_k, ambiguity_ratio=ambiguity_ratio,
            registration=registration,
        )

    def _landmark_conflict(self, candidate) -> str:
        """A common landmark landing far from where the peer mapped it
        falsifies the candidate outright.

        The grid matcher aliases in this repetitive office (confident modes
        3-15 m from the truth); a single shared marker settles which mode is
        real, because under a wrong mode it lands whole rooms away.
        """
        if not self.common_landmarks:
            return ""
        limit = float(self.get_parameter("max_landmark_conflict_m").value)
        c, s = math.cos(candidate[2]), math.sin(candidate[2])
        for tid in self.common_landmarks:
            p1 = self.leo1_landmarks.get(tid)
            p2 = self.leo2_landmarks.get(tid)
            if p1 is None or p2 is None:
                continue
            tx = candidate[0] + c * p2[0] - s * p2[1]
            ty = candidate[1] + s * p2[0] + c * p2[1]
            d = math.hypot(tx - p1[0], ty - p1[1])
            if d > limit:
                return (
                    f"common landmark {tid} lands {d:.1f} m from the peer's "
                    f"position under this candidate (> {limit:.1f} m)")
        return ""

    def _known_m2(self, grid) -> float:
        if grid is None:
            return 0.0
        values = np.asarray(grid.data, dtype=np.int8)
        res = float(grid.info.resolution)
        return float(np.count_nonzero(values >= 0)) * res * res

    def _preflight_reject(self, result: GridMatchResult, is_ambiguous: bool,
                          candidate, registration) -> str:
        mode = self._mode()
        if not result.success:
            return result.message
        min_first = float(
            self.get_parameter("min_known_m2_for_first_lock").value)
        # Two agreeing shared landmarks pin the transform on their own; the
        # size gate then only delays a well-witnessed merge (husarion v4:
        # a 0.08 m candidate with 2 witnesses was withheld a whole run
        # because one rover's map stayed at 53 m2).
        if (self.state.accepted is None and min_first > 0
                and len(self.common_landmarks) < 2):
            a1, a2 = self._known_m2(self.map1), self._known_m2(self.map2)
            if min(a1, a2) < min_first:
                return (
                    f"maps too small for a first lock ({a1:.0f}/{a2:.0f} m2 "
                    f"known < {min_first:.0f} m2): small half-maps of a "
                    f"symmetric office match their own mirror image")
        # Grid geometry alone repeatedly locked confident wrong modes in this
        # building (mirror at ~30 m2, +3 m alias at ~80 m2 interior). The
        # first acceptance needs one independent witness: a landmark both
        # rovers have seen, landing in the same place under the candidate.
        conflict = self._landmark_conflict(candidate)
        if conflict:
            return conflict
        if self.state.accepted is None and not self.common_landmarks:
            return ("first lock needs a landmark both rovers have seen; "
                    "grid-only matches alias in this repetitive building")
        if registration['wall_hit'] < float(
                self.get_parameter("min_refined_wall_hit").value):
            return (
                f"refined wall hit {registration['wall_hit']:.3f} < "
                f"min_refined_wall_hit"
            )
        if registration['refinement_translation_m'] > float(
                self.get_parameter("max_refinement_translation_m").value):
            return "local refinement escaped its translation trust region"
        if registration['refinement_yaw_deg'] > float(
                self.get_parameter("max_refinement_yaw_deg").value):
            return "local refinement escaped its yaw trust region"
        if result.free_space_conflict_ratio > float(
            self.get_parameter("max_free_space_conflict_ratio").value
        ):
            return (
                f"free-space conflict {result.free_space_conflict_ratio:.2f} > "
                f"max_free_space_conflict_ratio"
            )
        evidence_reject = fusion_evidence_rejection(
            mode,
            self.tag_estimate,
            candidate,
            is_ambiguous=is_ambiguous,
            max_translation_m=float(
                self.get_parameter("max_tag_map_disagreement_m").value
            ),
            max_yaw=math.radians(
                float(self.get_parameter("max_tag_map_disagreement_yaw_deg").value)
            ),
        )
        if evidence_reject:
            return evidence_reject
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
        registration: Optional[dict] = None,
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
        residual, geom_reject = self._geometry_gate(candidate)
        geometry_aligned = not bool(geom_reject)
        # Geometry is a mandatory final gate, but never erases a preflight
        # rejection. In particular, a symmetric office overlay must remain a
        # preview until independent landmark evidence selects the same mode.
        if geom_reject:
            extra_reject = extra_reject or geom_reject

        base_min = float(self.get_parameter("min_alignment_confidence").value)
        map_min = float(self.get_parameter("map_mode_min_confidence").value)
        min_conf = min_acceptance_confidence(
            len(self.common_landmarks),
            mode,
            overlap,
            is_ambiguous=is_ambiguous,
            tag_map_agreement=agreement,
            geometry_aligned=geometry_aligned,
            base_min=base_min,
            map_mode_min=map_min,
        )
        self.state.min_alignment_confidence = min_conf

        policy_preview = build_policy_debug(
            mode=mode,
            final_confidence=confidence,
            refined_wall_hit=(registration or {}).get('wall_hit'),
            refined_forward_hit=(registration or {}).get('forward_hit'),
            refined_reverse_hit=(registration or {}).get('reverse_hit'),
            seed_wall_hit=(registration or {}).get('seed_wall_hit'),
            refinement_translation_m=(registration or {}).get(
                'refinement_translation_m'),
            refinement_yaw_deg=(registration or {}).get('refinement_yaw_deg'),
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

        residual_m = residual.get("median_m")
        # Re-evaluate the accepted pose on the *current synchronized maps*.
        # The residual saved when it first locked becomes stale as each rover
        # reveals new walls. If the old pose now fails geometry while this
        # refined candidate passes, geometry—not an unrelated confidence
        # increment—must authorize the replacement.
        if self.state.accepted is not None:
            accepted_now, accepted_geom_reject = self._geometry_gate(
                self.state.accepted)
            accepted_now_m = accepted_now.get("median_m")
            if accepted_now_m is not None:
                self.state.accepted_residual_m = float(accepted_now_m)
            if geometry_aligned and accepted_geom_reject:
                self.state.accepted_residual_m = float("inf")
        if extra_reject:
            accepted, reason = False, extra_reject
            self.state.evaluate_candidate(
                candidate, confidence, extra_reject_reason=extra_reject)
        else:
            accepted, reason = self.state.evaluate_candidate(
                candidate, confidence, residual_m=residual_m)
        if accepted:
            self._accepted_evidence = {
                "common_landmark_count": len(self.common_landmarks),
                "tag_map_agreement": bool(agreement),
                "ambiguity_score": float(ambiguity_ratio),
                "is_ambiguous": bool(is_ambiguous),
            }

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
        policy.pop("reason", None)
        debug.update(policy)
        debug["reason"] = reason
        debug["accepted"] = accepted
        debug.update({
            "occupancy_residual_m": residual.get("median_m"),
            "occupancy_residual_p90_m": residual.get("p90_m"),
            "undilated_wall_hit": residual.get("undilated_hit"),
            "occupancy_overlap_fraction": residual.get("overlap_fraction"),
            "geometry_aligned": geometry_aligned,
        })
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
        else:
            self.get_logger().warn(log)

        # Keep accepted transform and accepted confidence synchronized. A
        # rejected candidate's confidence must never unlock an older pose.
        if self.state.accepted is not None:
            self._publish_transform(self.accepted_pub, self.state.accepted)
            self.accepted_confidence_pub.publish(Float32(
                data=float(self.state.accepted_confidence)))
            accepted_residual, accepted_reject = self._geometry_gate(
                self.state.accepted)
            accepted_median = accepted_residual.get("median_m")
            accepted_ok = not bool(accepted_reject)
            self.residual_pub.publish(Float32(
                data=99.0 if accepted_median is None
                else float(accepted_median)))
            self.geometry_ok_pub.publish(Bool(data=accepted_ok))
            self.accepted_validation_pub.publish(String(data=json.dumps({
                "dx": self.state.accepted[0],
                "dy": self.state.accepted[1],
                "yaw": self.state.accepted[2],
                "confidence": self.state.accepted_confidence,
                "residual_m": accepted_median,
                "residual_p90_m": accepted_residual.get("p90_m"),
                "undilated_wall_hit": accepted_residual.get(
                    "undilated_hit"),
                "overlap_fraction": accepted_residual.get(
                    "overlap_fraction"),
                "geometry_ok": accepted_ok,
                **self._accepted_evidence,
                "reason": accepted_reject or "geometry aligned",
            })))
        else:
            self.residual_pub.publish(Float32(data=99.0))
            self.geometry_ok_pub.publish(Bool(data=False))

        if recovery:
            payload = json.dumps(recovery)
            if payload != self._last_recovery:
                self._last_recovery = payload
                self.recovery_pub.publish(String(data=payload))
        elif self._last_recovery is not None:
            self._last_recovery = None

    def _geometry_gate(self, candidate):
        """Refuse a lock when occupancy walls remain visibly offset."""
        empty = {"median_m": None, "p90_m": None, "undilated_hit": None}
        if self.map1 is None or self.map2 is None:
            return empty, "waiting for both occupancy maps"
        g1, i1 = self._grid_tuple(self.map1)
        g2, i2 = self._grid_tuple(self.map2)
        stats = residual_stats(g1, i1, g2, i2, candidate)
        ok, why = geometric_lock_ok(
            stats,
            max_median_m=float(self.get_parameter("max_lock_residual_m").value),
            max_p90_m=float(self.get_parameter("max_lock_residual_p90_m").value),
            min_undilated_hit=float(
                self.get_parameter("min_undilated_wall_hit").value),
        )
        return stats, ("" if ok else why)


    def _search_window(self):
        # An accepted pose is the strongest prior: track it. But every Nth
        # cycle - and immediately when the accepted pose has stopped fitting
        # the current maps - run the full global window instead, so a wrong
        # early lock can be challenged and superseded by the true mode.
        # The tag hint is only trusted when the tag pipeline itself has
        # confidence in it - a zero-confidence estimate once parked the
        # search in a 2 m box around an 8 m error for seven minutes.
        mode = self._mode()
        self._cycle_count += 1
        self._global_recheck = False
        if self.state.accepted is not None:
            every_n = max(1, int(
                self.get_parameter("global_recheck_every_n").value))
            accepted_broken = (
                self.state.accepted_residual_m is not None
                and math.isinf(self.state.accepted_residual_m))
            if accepted_broken or self._cycle_count % every_n == 0:
                self._global_recheck = True
                return (
                    (0.0, 0.0, 0.0),
                    float(self.get_parameter("map_search_range_xy").value),
                    float(self.get_parameter("map_search_range_yaw").value),
                )
            return (
                self.state.accepted,
                float(self.get_parameter("hybrid_search_range_xy").value),
                float(self.get_parameter("hybrid_search_range_yaw").value),
            )
        if (mode in ("hybrid", "tag") and self.tag_estimate is not None
                and (self.tag_confidence or 0.0) >= 0.2):
            return (
                self.tag_estimate,
                float(self.get_parameter("hybrid_search_range_xy").value),
                float(self.get_parameter("hybrid_search_range_yaw").value),
            )
        # No trustworthy prior: full map search for map-only alignment.
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
