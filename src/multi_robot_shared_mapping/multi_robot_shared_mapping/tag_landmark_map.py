#!/usr/bin/env python3
"""
Persistent AprilTag landmark map (pure Python, no ROS imports).

Each robot keeps one LandmarkMap in its own map frame. Landmarks are NEVER
deleted because they are out of view; they are refined every time the same
tag is re-observed:
- position: incremental weighted mean (new sample weight 1/(n+1)), so noisy
  single detections cannot drag a well-observed landmark far.
- variance: running estimate of squared innovation (simple isotropic
  uncertainty in m^2).
- gating: once a landmark is mature (>= gate_min_observations), measurements
  that jump farther than gate_distance are rejected and only decay
  confidence slightly (bad detections must not overwrite good landmarks).

Also provides pure helpers to merge two landmark maps into a shared frame
and to compute an alignment confidence score, both unit-testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class TagLandmark:
    tag_id: int
    x: float
    y: float
    yaw: float
    first_seen_time: float
    last_seen_time: float
    observation_count: int = 1
    confidence: float = 0.1
    position_variance: float = 0.05  # isotropic uncertainty estimate, m^2
    rejected_count: int = 0

    def to_dict(self) -> Dict:
        return {
            "tag_id": self.tag_id,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
            "confidence": round(self.confidence, 3),
            "observation_count": self.observation_count,
            "position_variance": round(self.position_variance, 5),
            "first_seen_time": round(self.first_seen_time, 2),
            "last_seen_time": round(self.last_seen_time, 2),
        }


@dataclass
class LandmarkMap:
    """Persistent tag_id -> TagLandmark store with outlier gating."""

    gate_distance: float = 1.0
    gate_min_observations: int = 5
    landmarks: Dict[int, TagLandmark] = field(default_factory=dict)

    def update(self, tag_id: int, x: float, y: float, yaw: float, stamp: float) -> str:
        """Insert or refine a landmark. Returns 'added', 'updated' or 'rejected'."""
        existing = self.landmarks.get(tag_id)
        if existing is None:
            self.landmarks[tag_id] = TagLandmark(
                tag_id=tag_id, x=x, y=y, yaw=yaw,
                first_seen_time=stamp, last_seen_time=stamp,
            )
            self._refresh_confidence(self.landmarks[tag_id])
            return "added"

        innovation = math.hypot(x - existing.x, y - existing.y)
        mature = existing.observation_count >= self.gate_min_observations
        if mature and innovation > self.gate_distance:
            # Outlier against a well-observed landmark: keep position, decay
            # confidence so persistent disagreement eventually shows up.
            existing.rejected_count += 1
            existing.confidence = max(0.05, existing.confidence * 0.95)
            return "rejected"

        weight = 1.0 / (existing.observation_count + 1)
        existing.x += weight * (x - existing.x)
        existing.y += weight * (y - existing.y)
        existing.yaw = _normalize_angle(
            existing.yaw + weight * _normalize_angle(yaw - existing.yaw)
        )
        existing.position_variance = (
            (1.0 - weight) * existing.position_variance + weight * innovation ** 2
        )
        existing.observation_count += 1
        existing.last_seen_time = stamp
        self._refresh_confidence(existing)
        return "updated"

    def _refresh_confidence(self, landmark: TagLandmark):
        count_factor = min(1.0, landmark.observation_count / 10.0)
        noise_factor = 1.0 / (1.0 + math.sqrt(max(0.0, landmark.position_variance)))
        landmark.confidence = max(0.05, count_factor * noise_factor)

    def positions(self) -> Dict[int, Tuple[float, float]]:
        return {tid: (lm.x, lm.y) for tid, lm in self.landmarks.items()}

    def to_dict_list(self) -> List[Dict]:
        return [self.landmarks[tid].to_dict() for tid in sorted(self.landmarks)]


def geometric_spread(points: Sequence[Tuple[float, float]]) -> float:
    """RMS distance of points from their centroid (0 for identical points)."""
    if len(points) < 2:
        return 0.0
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    return math.sqrt(
        sum((p[0] - cx) ** 2 + (p[1] - cy) ** 2 for p in points) / len(points)
    )


def compute_tag_alignment_confidence(
    common_ids: Sequence[int],
    map1: LandmarkMap,
    map2: LandmarkMap,
    mean_residual: float,
    max_residual: float,
    previous_transform: Optional[Tuple[float, float, float]],
    current_transform: Tuple[float, float, float],
) -> float:
    """
    Confidence in [0, 1] from: number of common tags, per-tag observation
    counts, residual error, geometric spread and transform stability.
    """
    if not common_ids:
        return 0.0

    count_factor = min(1.0, len(common_ids) / 4.0)

    obs_counts = [
        min(map1.landmarks[tid].observation_count, map2.landmarks[tid].observation_count)
        for tid in common_ids
    ]
    observation_factor = min(1.0, (sum(obs_counts) / len(obs_counts)) / 10.0)

    residual_factor = 1.0 / (1.0 + 5.0 * mean_residual + 2.0 * max_residual)

    spread = geometric_spread([
        (map1.landmarks[tid].x, map1.landmarks[tid].y) for tid in common_ids
    ])
    spread_factor = min(1.0, spread / 2.0)

    if previous_transform is None:
        stability_factor = 0.5
    else:
        jump = math.hypot(
            current_transform[0] - previous_transform[0],
            current_transform[1] - previous_transform[1],
        )
        stability_factor = 1.0 / (1.0 + jump)

    confidence = (
        0.30 * count_factor
        + 0.15 * observation_factor
        + 0.25 * residual_factor
        + 0.15 * spread_factor
        + 0.15 * stability_factor
    )
    return max(0.0, min(1.0, confidence))


def merge_landmark_dicts(
    landmarks1: Sequence[Dict],
    landmarks2: Sequence[Dict],
    transform_2_to_1: Tuple[float, float, float],
) -> List[Dict]:
    """
    Merge robot2 landmark dicts (transformed into robot1's map frame) with
    robot1 landmark dicts (as produced by LandmarkMap.to_dict_list()).
    Common tags are combined with a confidence-weighted position average.
    """
    dx, dy, yaw = transform_2_to_1
    c, s = math.cos(yaw), math.sin(yaw)

    merged: Dict[int, Dict] = {lm["tag_id"]: dict(lm) for lm in landmarks1}

    for lm in landmarks2:
        tx = c * lm["x"] - s * lm["y"] + dx
        ty = s * lm["x"] + c * lm["y"] + dy
        tyaw = _normalize_angle(lm["yaw"] + yaw)

        tid = lm["tag_id"]
        if tid not in merged:
            entry = dict(lm)
            entry.update({"x": round(tx, 4), "y": round(ty, 4), "yaw": round(tyaw, 4)})
            merged[tid] = entry
            continue

        base = merged[tid]
        w1 = max(1e-6, base["confidence"])
        w2 = max(1e-6, lm["confidence"])
        total = w1 + w2
        base["x"] = round((w1 * base["x"] + w2 * tx) / total, 4)
        base["y"] = round((w1 * base["y"] + w2 * ty) / total, 4)
        base["yaw"] = round(_normalize_angle(
            base["yaw"] + (w2 / total) * _normalize_angle(tyaw - base["yaw"])
        ), 4)
        base["observation_count"] += lm["observation_count"]
        base["confidence"] = round(max(base["confidence"], lm["confidence"]), 3)

    return [merged[tid] for tid in sorted(merged)]
