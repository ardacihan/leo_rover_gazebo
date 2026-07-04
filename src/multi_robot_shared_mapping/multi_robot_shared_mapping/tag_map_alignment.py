#!/usr/bin/env python3
"""
Pure 2D rigid-transform estimation for tag-based map alignment.

If SLAM maps are corrupted (e.g. robot pushing into a wall), tag alignment
estimates will also be unreliable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

Point2D = Tuple[float, float]


@dataclass
class TransformEstimate:
    dx: float
    dy: float
    yaw: float
    success: bool
    confidence: float
    mean_reprojection_error: float
    num_tags: int
    translation_error: Optional[float] = None
    yaw_error: Optional[float] = None
    message: str = ""


def apply_2d_transform(x: float, y: float, dx: float, dy: float, yaw: float) -> Point2D:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y + dx, s * x + c * y + dy


def _as_array(points: Sequence[Point2D]) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("Points must be Nx2")
    return arr


def estimate_2d_transform(
    source_points: Sequence[Point2D],
    target_points: Sequence[Point2D],
    *,
    min_tags: int = 2,
    max_mean_error: float = 0.35,
    max_point_error: float = 0.50,
    ground_truth: Optional[Tuple[float, float, float]] = None,
) -> TransformEstimate:
    """
    Estimate planar transform mapping source_points -> target_points.

    source_points are tag positions expressed in leo2/map.
    target_points are the same tag IDs expressed in leo1/map.
    """
    if len(source_points) != len(target_points):
        return TransformEstimate(
            dx=0.0,
            dy=0.0,
            yaw=0.0,
            success=False,
            confidence=0.0,
            mean_reprojection_error=float("inf"),
            num_tags=min(len(source_points), len(target_points)),
            message="Source and target point counts differ",
        )

    num_tags = len(source_points)
    if num_tags < min_tags:
        return TransformEstimate(
            dx=0.0,
            dy=0.0,
            yaw=0.0,
            success=False,
            confidence=0.0,
            mean_reprojection_error=float("inf"),
            num_tags=num_tags,
            message=f"Need at least {min_tags} matched tags, got {num_tags}",
        )

    src = _as_array(source_points)
    tgt = _as_array(target_points)

    src_centroid = src.mean(axis=0)
    tgt_centroid = tgt.mean(axis=0)
    src_centered = src - src_centroid
    tgt_centered = tgt - tgt_centroid

    h = src_centered.T @ tgt_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T

    yaw = math.atan2(r[1, 0], r[0, 0])
    t = tgt_centroid - r @ src_centroid
    dx = float(t[0])
    dy = float(t[1])

    transformed = (r @ src.T).T + t
    errors = np.linalg.norm(transformed - tgt, axis=1)
    mean_error = float(errors.mean())
    max_error = float(errors.max())

    confidence = min(1.0, num_tags / 4.0)
    if num_tags == 2:
        confidence *= 0.6

    translation_error = None
    yaw_error = None
    if ground_truth is not None:
        gt_dx, gt_dy, gt_yaw = ground_truth
        translation_error = math.hypot(dx - gt_dx, dy - gt_dy)
        yaw_error = abs(_normalize_angle(yaw - gt_yaw))

    if mean_error > max_mean_error or max_error > max_point_error:
        return TransformEstimate(
            dx=dx,
            dy=dy,
            yaw=yaw,
            success=False,
            confidence=confidence,
            mean_reprojection_error=mean_error,
            num_tags=num_tags,
            translation_error=translation_error,
            yaw_error=yaw_error,
            message=(
                f"Rejected transform: mean error={mean_error:.3f} m, "
                f"max error={max_error:.3f} m"
            ),
        )

    return TransformEstimate(
        dx=dx,
        dy=dy,
        yaw=yaw,
        success=True,
        confidence=confidence,
        mean_reprojection_error=mean_error,
        num_tags=num_tags,
        translation_error=translation_error,
        yaw_error=yaw_error,
        message="Transform accepted",
    )


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def estimate_transform_from_single_tag(
    target_x: float,
    target_y: float,
    target_yaw: float,
    source_x: float,
    source_y: float,
    source_yaw: float,
) -> Tuple[float, float, float]:
    """
    Weak single-landmark relocalization hint: one tag seen by both robots.

    The tag orientation difference fixes yaw, then translation follows from
    the positions. With only one landmark this is inherently low confidence
    (tag yaw noise translates directly into position error at range), so the
    caller must cap the confidence accordingly.

    Returns (dx, dy, yaw) mapping source (leo2/map) into target (leo1/map).
    """
    yaw = _normalize_angle(target_yaw - source_yaw)
    c = math.cos(yaw)
    s = math.sin(yaw)
    dx = target_x - (c * source_x - s * source_y)
    dy = target_y - (s * source_x + c * source_y)
    return dx, dy, yaw


def transform_points(
    points: Iterable[Point2D],
    dx: float,
    dy: float,
    yaw: float,
) -> List[Point2D]:
    return [apply_2d_transform(x, y, dx, dy, yaw) for x, y in points]
