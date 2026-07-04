#!/usr/bin/env python3
"""
Collaborative exploration policy (no ROS imports).

Robots should explore separate frontiers by default. Recovery actions
(revisit tags, overlap areas) are occasional — not required before exploration.

Confidence tiers replace hard "wait for 2+ common tags" blocking:
- no common tags: map-only matching, medium confidence if overlap strong
- one tag: weak/medium anchor; accept when map matching agrees
- two tags: stronger when spread is good
- three+: high confidence with residual checks
"""

from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

ConfidenceLevel = str  # "low" | "medium" | "high"


def tag_map_agree(
    tag_estimate: Optional[Tuple[float, float, float]],
    map_estimate: Tuple[float, float, float],
    max_translation_m: float,
    max_yaw: float,
) -> bool:
    """True when tag hint and map match describe the same transform."""
    if tag_estimate is None:
        return False
    translation = math.hypot(
        tag_estimate[0] - map_estimate[0], tag_estimate[1] - map_estimate[1]
    )
    yaw_delta = abs(_normalize_angle(tag_estimate[2] - map_estimate[2]))
    return translation <= max_translation_m and yaw_delta <= max_yaw


def classify_confidence_level(
    final_confidence: float,
    common_landmark_count: int,
    map_overlap_score: float,
    ambiguity_score: float,
    *,
    tag_map_agreement: bool = False,
    is_ambiguous: bool = False,
) -> ConfidenceLevel:
    """
    Map numeric confidence + context into low / medium / high.

    ambiguity_score: second_best / best (higher = more ambiguous).
    """
    if is_ambiguous or ambiguity_score >= 0.85:
        return "low"
    if final_confidence < 0.25:
        return "low"

    if common_landmark_count >= 3 and final_confidence >= 0.65:
        return "high"
    if common_landmark_count >= 2 and final_confidence >= 0.55:
        return "high"
    if (
        common_landmark_count == 1
        and tag_map_agreement
        and map_overlap_score >= 0.30
        and final_confidence >= 0.40
        and not is_ambiguous
    ):
        return "medium"
    if (
        common_landmark_count == 0
        and map_overlap_score >= 0.40
        and final_confidence >= 0.50
        and not is_ambiguous
    ):
        return "medium"
    if final_confidence >= 0.70 and not is_ambiguous:
        if common_landmark_count <= 1 and not tag_map_agreement:
            return "medium"
        return "high"
    if final_confidence >= 0.40:
        return "medium"
    return "low"


def min_acceptance_confidence(
    common_landmark_count: int,
    mode: str,
    map_overlap_score: float,
    *,
    is_ambiguous: bool = False,
    tag_map_agreement: bool = False,
    base_min: float = 0.5,
    map_mode_min: float = 0.6,
) -> float:
    """
    Dynamic acceptance floor — lower when map/tag evidence is strong.

    One tag alone is never high-confidence; acceptance requires map agreement
    or strong occupancy overlap depending on mode.
    """
    if is_ambiguous:
        return 1.0  # never accept ambiguous transforms

    if common_landmark_count >= 3:
        return max(0.40, base_min - 0.10)
    if common_landmark_count >= 2:
        return max(0.42, base_min - 0.08)

    if common_landmark_count == 1:
        if tag_map_agreement and map_overlap_score >= 0.25:
            return max(0.38, base_min - 0.12)
        return max(0.50, base_min)  # tag alone: harder to accept

    # No common tags — map-only path.
    if mode in ("map", "hybrid"):
        if map_overlap_score >= 0.45 and not is_ambiguous:
            return max(0.42, map_mode_min - 0.18)
        if map_overlap_score >= 0.35:
            return map_mode_min - 0.05
        return map_mode_min

    return base_min


def exploration_allowed(
    confidence_level: ConfidenceLevel,
    *,
    is_ambiguous: bool = False,
    leo1_quality: float = 1.0,
    leo2_quality: float = 1.0,
    local_map_quality_min: float = 0.15,
) -> bool:
    """
    True when robots should explore separate frontiers normally.

    Recovery (revisit tags/overlap) is only needed when this is False.
    """
    if leo1_quality < local_map_quality_min or leo2_quality < local_map_quality_min:
        return False
    if is_ambiguous:
        return False
    return confidence_level in ("medium", "high")


def recommended_exploration_action(
    confidence_level: ConfidenceLevel,
    exploration_ok: bool,
    recovery: Optional[dict],
) -> str:
    """Primary action hint for operators / future frontier planners."""
    if recovery is not None:
        return str(recovery.get("recommended_action", "recovery"))
    if exploration_ok:
        return "explore_separate_frontiers"
    return "hold_and_monitor"


def build_policy_debug(
    *,
    mode: str,
    final_confidence: float,
    common_landmark_count: int,
    map_overlap_score: float,
    ambiguity_score: float,
    is_ambiguous: bool,
    tag_map_agreement: bool,
    leo1_quality: float,
    leo2_quality: float,
    recovery: Optional[dict],
    base_min: float = 0.5,
    map_mode_min: float = 0.6,
) -> Dict:
    """Fields merged into /alignment_debug_json."""
    level = classify_confidence_level(
        final_confidence,
        common_landmark_count,
        map_overlap_score,
        ambiguity_score,
        tag_map_agreement=tag_map_agreement,
        is_ambiguous=is_ambiguous,
    )
    explore_ok = exploration_allowed(
        level,
        is_ambiguous=is_ambiguous,
        leo1_quality=leo1_quality,
        leo2_quality=leo2_quality,
    )
    min_conf = min_acceptance_confidence(
        common_landmark_count,
        mode,
        map_overlap_score,
        is_ambiguous=is_ambiguous,
        tag_map_agreement=tag_map_agreement,
        base_min=base_min,
        map_mode_min=map_mode_min,
    )
    reason = _exploration_reason(
        level, explore_ok, common_landmark_count, is_ambiguous, recovery
    )
    return {
        "common_landmark_count": common_landmark_count,
        "map_overlap_score": map_overlap_score,
        "ambiguity_score": ambiguity_score,
        "confidence_level": level,
        "shared_map_confidence_level": level,
        "exploration_allowed": explore_ok,
        "min_acceptance_confidence": min_conf,
        "tag_map_agreement": tag_map_agreement,
        "recommended_action": recommended_exploration_action(
            level, explore_ok, recovery
        ),
        "reason": reason,
    }


def _exploration_reason(
    level: ConfidenceLevel,
    explore_ok: bool,
    common_count: int,
    is_ambiguous: bool,
    recovery: Optional[dict],
) -> str:
    if recovery is not None:
        return str(recovery.get("reason", "Recovery recommended"))
    if explore_ok:
        if common_count == 0:
            return (
                "Map-only alignment sufficient for collaborative exploration; "
                "robots should take separate frontiers."
            )
        if common_count == 1:
            return (
                "One common tag anchors alignment at medium confidence; "
                "robots may explore separately."
            )
        return "Alignment confidence supports separate frontier exploration."
    if is_ambiguous:
        return (
            "Map matching ambiguous; keep candidate map, explore distinctive "
            "areas or revisit overlap only if needed."
        )
    if level == "low":
        return (
            "Low alignment confidence; leo1-only shared map or candidate preview. "
            "Occasional recovery may help — not required for all exploration."
        )
    return "Monitoring alignment."


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
