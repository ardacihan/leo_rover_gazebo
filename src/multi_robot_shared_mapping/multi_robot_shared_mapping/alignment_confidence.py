#!/usr/bin/env python3
"""
Pure alignment-confidence scoring and ambiguity detection (no ROS imports).

final confidence combines (weights renormalized over available components):
- occupancy_overlap_score: fraction of leo2 walls landing on leo1 walls.
- free_space_conflict_score: 1 - conflict ratio (walls inside known free space).
- tag_alignment_confidence: confidence reported by the tag aligner.
- tag_residual_score: 1 / (1 + residual) of tag landmarks after transform.
- common_landmark_count_score / landmark_spread_score: landmark geometry.
- transform_stability_score: how close the candidate is to the accepted one.
- ambiguity_score (input as "unambiguity"): 1 when the best map-match beats
  the runner-up clearly, near 0 when two different transforms explain the
  overlap equally well (symmetric corridors etc.).
- local_map_quality_score: min of both robots' local map quality (a corrupted
  local SLAM map makes any alignment untrustworthy).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Candidate = Tuple[float, float, float, float]  # dx, dy, yaw, score


@dataclass
class ConfidenceInputs:
    occupancy_overlap_score: float
    free_space_conflict_score: float
    transform_stability_score: float
    unambiguity_score: float
    local_map_quality_score: float
    tag_alignment_confidence: Optional[float] = None
    tag_residual_score: Optional[float] = None
    common_landmark_count_score: Optional[float] = None
    landmark_spread_score: Optional[float] = None


# (weight, attribute) pairs; optional attributes are skipped when None.
_WEIGHTS = (
    (0.30, "occupancy_overlap_score"),
    (0.15, "free_space_conflict_score"),
    (0.10, "transform_stability_score"),
    (0.15, "unambiguity_score"),
    (0.10, "local_map_quality_score"),
    (0.10, "tag_alignment_confidence"),
    (0.05, "tag_residual_score"),
    (0.025, "common_landmark_count_score"),
    (0.025, "landmark_spread_score"),
)


def compute_final_confidence(inputs: ConfidenceInputs) -> float:
    """Weighted mean over available components, clipped to [0, 1]."""
    total_weight = 0.0
    total = 0.0
    for weight, name in _WEIGHTS:
        value = getattr(inputs, name)
        if value is None:
            continue
        total_weight += weight
        total += weight * max(0.0, min(1.0, float(value)))
    if total_weight <= 0.0:
        return 0.0
    return max(0.0, min(1.0, total / total_weight))


def detect_ambiguity(
    candidates: Sequence[Candidate],
    *,
    min_candidate_separation_m: float = 1.0,
    min_candidate_separation_yaw: float = math.radians(15.0),
    ambiguity_ratio_threshold: float = 0.85,
) -> Tuple[float, bool]:
    """
    Ambiguity from top-K map-matching candidates.

    Returns (ambiguity_ratio, is_ambiguous). ambiguity_ratio is
    second_best_score / best_score considering only candidates that describe
    a genuinely DIFFERENT transform (farther than the separation thresholds
    from the best). Nearby candidates are just the same optimum sampled twice.
    """
    if len(candidates) < 2:
        return 0.0, False

    ranked = sorted(candidates, key=lambda c: c[3], reverse=True)
    best = ranked[0]
    if best[3] <= 0:
        return 1.0, True

    for other in ranked[1:]:
        distance = math.hypot(other[0] - best[0], other[1] - best[1])
        yaw_delta = abs(_normalize_angle(other[2] - best[2]))
        if distance < min_candidate_separation_m and yaw_delta < min_candidate_separation_yaw:
            continue
        ratio = max(0.0, other[3] / best[3])
        return ratio, ratio >= ambiguity_ratio_threshold

    return 0.0, False


def select_top_candidates(
    candidates: Sequence[Candidate],
    k: int = 5,
    min_separation_m: float = 0.5,
    min_separation_yaw: float = math.radians(10.0),
) -> List[Candidate]:
    """Non-maximum suppression: keep the k best mutually distinct candidates."""
    ranked = sorted(candidates, key=lambda c: c[3], reverse=True)
    kept: List[Candidate] = []
    for cand in ranked:
        distinct = all(
            math.hypot(cand[0] - kept_c[0], cand[1] - kept_c[1]) >= min_separation_m
            or abs(_normalize_angle(cand[2] - kept_c[2])) >= min_separation_yaw
            for kept_c in kept
        )
        if distinct:
            kept.append(cand)
        if len(kept) >= k:
            break
    return kept


def transforms_disagree(
    transform_a: Tuple[float, float, float],
    transform_b: Tuple[float, float, float],
    max_translation_m: float,
    max_yaw: float,
) -> bool:
    """True when two transform estimates differ beyond the given thresholds."""
    translation = math.hypot(
        transform_a[0] - transform_b[0], transform_a[1] - transform_b[1]
    )
    yaw_delta = abs(_normalize_angle(transform_a[2] - transform_b[2]))
    return translation > max_translation_m or yaw_delta > max_yaw


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
