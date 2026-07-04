#!/usr/bin/env python3
"""
Occasional recovery recommendations when exploration cannot proceed safely.

Recovery is NOT required for normal collaborative exploration. Robots should
explore separate frontiers unless confidence is low, map matching is ambiguous,
or local map quality is poor.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple


def recommend_recovery(
    *,
    confidence: float,
    min_confidence: float,
    common_ids: Sequence[int],
    leo1_landmarks: Dict[int, Tuple[float, float]],
    leo2_landmarks: Dict[int, Tuple[float, float]],
    leo1_pose: Optional[Tuple[float, float]] = None,
    leo2_pose: Optional[Tuple[float, float]] = None,
    is_ambiguous: bool = False,
    leo1_quality: float = 1.0,
    leo2_quality: float = 1.0,
    reset_bad_local_map_recommended: bool = False,
    exploration_allowed: bool = True,
    map_overlap_score: float = 0.0,
) -> Optional[dict]:
    """
    Return a recovery JSON dict, or None when exploration can continue.

    Recovery is triggered only for genuine problems — not merely because fewer
    than two common AprilTags exist.
    """
    if exploration_allowed and confidence >= min_confidence * 0.85:
        return None

    if leo1_quality < 0.15:
        return _slam_recovery("leo1", reset_bad_local_map_recommended)
    if leo2_quality < 0.15:
        return _slam_recovery("leo2", reset_bad_local_map_recommended)

    if is_ambiguous:
        return {
            "status": "low_confidence",
            "recommended_robot": "leo2",
            "recommended_action": "go_to_distinctive_area",
            "target_tag_id": None,
            "target_x": None,
            "target_y": None,
            "reason": (
                "Map matching ambiguous; occasionally move one robot to a more "
                "distinctive area — not required for routine separate exploration."
            ),
        }

    # No common tags: suggest overlap region, not mandatory tag collection.
    if len(common_ids) == 0:
        if map_overlap_score >= 0.20:
            return {
                "status": "low_confidence",
                "recommended_robot": "leo2",
                "recommended_action": "explore_overlap_frontier",
                "target_tag_id": None,
                "target_x": None,
                "target_y": None,
                "reason": (
                    "No common tags yet; optional overlap scan may improve "
                    "map-only alignment while robots continue separate exploration."
                ),
            }
        return {
            "status": "low_confidence",
            "recommended_robot": "leo2",
            "recommended_action": "explore_separate_frontiers",
            "target_tag_id": None,
            "target_x": None,
            "target_y": None,
            "reason": (
                "Building initial maps independently; alignment will improve as "
                "occupancy overlap grows — no tag revisit required yet."
            ),
        }

    # One common tag with low confidence: optional landmark scan, not a blocker.
    if len(common_ids) == 1:
        leo1_only = set(leo1_landmarks) - set(leo2_landmarks)
        leo2_only = set(leo2_landmarks) - set(leo1_landmarks)
        if confidence < min_confidence and (leo2_only or leo1_only):
            if leo2_only:
                tid = _nearest_tag(leo2_pose, leo2_landmarks, leo2_only)
                x, y = leo2_landmarks[tid]
                robot = "leo2"
            else:
                tid = _nearest_tag(leo1_pose, leo1_landmarks, leo1_only)
                x, y = leo1_landmarks[tid]
                robot = "leo1"
            return {
                "status": "low_confidence",
                "recommended_robot": robot,
                "recommended_action": "go_to_landmark",
                "target_tag_id": tid,
                "target_x": x,
                "target_y": y,
                "reason": (
                    f"Optional recovery: {robot} may scan tag_{tid} to strengthen "
                    "alignment — robots can otherwise keep exploring separately."
                ),
            }
        return None

    # Two+ common tags but still low confidence: optional third tag or overlap.
    for robot, landmarks, pose, other in (
        ("leo2", leo2_landmarks, leo2_pose, leo1_landmarks),
        ("leo1", leo1_landmarks, leo1_pose, leo2_landmarks),
    ):
        missing = set(other) - set(landmarks)
        if missing and confidence < min_confidence:
            tid = _nearest_tag(pose, landmarks, missing, fallback=other)
            x, y = (other[tid] if tid in other else landmarks.get(tid, (0.0, 0.0)))
            return {
                "status": "low_confidence",
                "recommended_robot": robot,
                "recommended_action": "go_to_landmark",
                "target_tag_id": tid,
                "target_x": x,
                "target_y": y,
                "reason": (
                    "Optional: observe an additional landmark to improve spread — "
                    "not required for continued separate exploration."
                ),
            }

    if confidence < min_confidence:
        return {
            "status": "low_confidence",
            "recommended_robot": "leo2",
            "recommended_action": "rescan_overlap_region",
            "target_tag_id": None,
            "target_x": None,
            "target_y": None,
            "reason": (
                "Occasional overlap rescan may improve alignment; robots may "
                "otherwise continue mapping different areas."
            ),
        }

    return None


def _slam_recovery(robot: str, reset_recommended: bool) -> dict:
    suffix = (
        f" Restart/reinitialize local SLAM for {robot} or start a new submap."
        if reset_recommended
        else ""
    )
    return {
        "status": "low_confidence",
        "recommended_robot": robot,
        "recommended_action": "relocalize_or_reset_slam",
        "target_tag_id": None,
        "target_x": None,
        "target_y": None,
        "reason": (
            f"Local map quality low for {robot}; holding fusion and requesting "
            f"relocalization.{suffix}"
        ),
    }


def _nearest_tag(
    pose: Optional[Tuple[float, float]],
    local_landmarks: Dict[int, Tuple[float, float]],
    candidate_ids: set,
    fallback: Optional[Dict[int, Tuple[float, float]]] = None,
) -> int:
    ids = list(candidate_ids)
    if not ids:
        return 0
    if pose is None:
        return min(ids)
    px, py = pose

    def dist(tid):
        src = fallback if tid not in local_landmarks and fallback else local_landmarks
        if tid not in src and fallback and tid in fallback:
            lx, ly = fallback[tid]
        else:
            lx, ly = src.get(tid, (0.0, 0.0))
        return math.hypot(lx - px, ly - py)

    return min(ids, key=dist)
