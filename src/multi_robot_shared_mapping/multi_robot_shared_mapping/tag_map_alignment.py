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



# Residual below which the position fit is trusted outright, and above which
# the marker orientations carry their maximum share. Both in metres, chosen
# from the four 2026-08-24 runs: residual 0.11-0.22 m went with sub-degree
# position errors, 1.05 m went with a 7 deg one.
_ORI_RESID_LO = 0.25
_ORI_RESID_HI = 1.00
_ORI_MAX_WEIGHT = 0.70


def _circular_mean(angles):
    """Mean of angles, done on the circle so 179 and -179 average to 180."""
    if not len(angles):
        return 0.0
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def yaw_from_orientations(source_yaws, target_yaws, inlier_tol_rad=0.35):
    """Rotation implied by each marker's own orientation, outliers rejected.

    Every matched marker is the same physical plate, so target_yaw - source_yaw
    is the map-to-map rotation, once per marker and independent of where the
    markers sit. Returns (yaw, sigma, n_inliers, n_total) or None.

    The consensus step matters: a single marker whose pose flipped to the wrong
    branch of the planar-PnP ambiguity is 100+ degrees out and would drag a
    plain average with it.
    """
    if source_yaws is None or target_yaws is None:
        return None
    if len(source_yaws) != len(target_yaws) or not len(source_yaws):
        return None
    diffs = [_normalize_angle(t - s) for s, t in zip(source_yaws, target_yaws)]

    best = None
    for candidate in diffs:
        inliers = [d for d in diffs
                   if abs(_normalize_angle(d - candidate)) <= inlier_tol_rad]
        if best is None or len(inliers) > len(best):
            best = inliers
    if not best:
        return None

    yaw = _circular_mean(best)
    if len(best) > 1:
        var = sum(_normalize_angle(d - yaw) ** 2 for d in best) / (len(best) - 1)
        sigma = math.sqrt(var / len(best))
    else:
        # One marker agreeing with nothing is a weak vote, not a measurement.
        sigma = inlier_tol_rad
    return yaw, max(sigma, 0.01), len(best), len(diffs)

def estimate_2d_transform(
    source_points: Sequence[Point2D],
    target_points: Sequence[Point2D],
    *,
    min_tags: int = 2,
    max_mean_error: float = 0.35,
    max_point_error: float = 0.50,
    ground_truth: Optional[Tuple[float, float, float]] = None,
    source_yaws: Optional[Sequence[float]] = None,
    target_yaws: Optional[Sequence[float]] = None,
    use_orientation: bool = True,
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

    # --- fuse in the rotation the marker orientations imply -----------------
    # How much to trust the angle the positions imply is decided by the fit
    # RESIDUAL, not by any analytic sigma. Measured over four runs, the
    # first-order sigma is simply not calibrated -- office had a small sigma
    # and a 7.1 deg error, husarion a larger sigma and a 0.09 deg error --
    # because a warped map partly absorbs into the rigid fit. The residual
    # does track it: when the two landmark sets genuinely disagree about the
    # shape they form, the rotation they imply is not worth much, and the
    # per-marker orientations (which do not care about layout at all) are.
    yaw_note = ""
    if use_orientation:
        ori = yaw_from_orientations(source_yaws, target_yaws)
        resid = np.linalg.norm(((r @ src.T).T + t) - tgt, axis=1)
        resid_mean = float(resid.mean())
        if ori is not None:
            yaw_ori, sigma_ori, n_in, n_tot = ori
            enough = n_in >= 2 and n_in >= 0.5 * n_tot
            w = (resid_mean - _ORI_RESID_LO) / (_ORI_RESID_HI - _ORI_RESID_LO)
            w = max(0.0, min(w, _ORI_MAX_WEIGHT)) if enough else 0.0
            if w > 0.0:
                delta = _normalize_angle(yaw_ori - yaw)
                fused = _normalize_angle(yaw + delta * w)
                yaw_note = (
                    f"; yaw pos={math.degrees(yaw):.1f} "
                    f"ori={math.degrees(yaw_ori):.1f} "
                    f"({n_in}/{n_tot} inliers) residual={resid_mean:.2f} m "
                    f"-> weight {w:.2f} -> {math.degrees(fused):.1f} deg"
                )
                yaw = fused
                # Re-derive the translation for the rotation actually chosen.
                c, s = math.cos(yaw), math.sin(yaw)
                r = np.array([[c, -s], [s, c]])
                t = tgt_centroid - r @ src_centroid
            else:
                yaw_note = (
                    f"; positions agree (residual={resid_mean:.2f} m), "
                    f"orientation not needed"
                )

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
                f"max error={max_error:.3f} m{yaw_note}"
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
