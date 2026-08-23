"""Numerical primitives for sensor and odometry calibration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class SectorStatistics:
    count: int
    median: float
    minimum: float
    maximum: float
    standard_deviation: float


@dataclass(frozen=True)
class PlaneFit:
    normal: np.ndarray
    offset: float
    distance: float
    rms_error: float
    inlier_count: int


@dataclass(frozen=True)
class LineFit2D:
    normal: np.ndarray
    offset: float
    distance: float
    normal_angle: float
    rms_error: float
    point_count: int


@dataclass(frozen=True)
class CameraLevel:
    roll_rad: float
    pitch_down_rad: float
    up_vector: np.ndarray


def sector_statistics(
    *,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    center_angle: float,
    half_width: float,
    minimum_range: float,
    maximum_range: float,
) -> SectorStatistics:
    if angle_increment == 0.0:
        raise ValueError("angle_increment must be non-zero")
    if half_width < 0.0:
        raise ValueError("half_width must be non-negative")
    values = []
    for index, raw in enumerate(ranges):
        angle = angle_min + index * angle_increment
        delta = normalize_angle(angle - center_angle)
        value = float(raw)
        if abs(delta) <= half_width and math.isfinite(value):
            if minimum_range <= value <= maximum_range:
                values.append(value)
    if not values:
        raise ValueError("sector contains no valid ranges")
    array = np.asarray(values, dtype=float)
    return SectorStatistics(
        count=int(array.size),
        median=float(np.median(array)),
        minimum=float(np.min(array)),
        maximum=float(np.max(array)),
        standard_deviation=float(np.std(array)),
    )


def _as_points(points: Iterable[Iterable[float]]) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("points must be an N x 3 array")
    array = array[np.isfinite(array).all(axis=1)]
    if array.shape[0] < 3:
        raise ValueError("at least three finite points are required")
    return array


def fit_plane_svd(points: Iterable[Iterable[float]]) -> PlaneFit:
    array = _as_points(points)
    centroid = np.mean(array, axis=0)
    _, _, vh = np.linalg.svd(array - centroid, full_matrices=False)
    normal = vh[-1].astype(float)
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        raise ValueError("plane normal is degenerate")
    normal /= norm
    offset = -float(np.dot(normal, centroid))
    residuals = array @ normal + offset
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    return PlaneFit(
        normal=normal,
        offset=offset,
        distance=abs(offset),
        rms_error=rms,
        inlier_count=int(array.shape[0]),
    )


def fit_plane_ransac(
    points: Iterable[Iterable[float]],
    *,
    distance_threshold: float = 0.02,
    iterations: int = 200,
    seed: int = 0,
    expected_normal: Iterable[float] | None = None,
    maximum_normal_angle: float = math.pi / 2.0,
) -> PlaneFit:
    array = _as_points(points)
    if distance_threshold <= 0.0:
        raise ValueError("distance_threshold must be positive")
    if iterations <= 0:
        raise ValueError("iterations must be positive")
    if not math.isfinite(maximum_normal_angle) or not 0.0 < maximum_normal_angle <= math.pi / 2.0:
        raise ValueError("maximum_normal_angle must be in (0, pi/2]")
    preferred = None
    minimum_alignment = 0.0
    if expected_normal is not None:
        preferred = np.asarray(expected_normal, dtype=float).reshape(3)
        if not np.isfinite(preferred).all():
            raise ValueError("expected_normal must be finite")
        norm = float(np.linalg.norm(preferred))
        if norm <= 1e-12:
            raise ValueError("expected_normal must be non-zero")
        preferred /= norm
        minimum_alignment = math.cos(maximum_normal_angle)
    rng = np.random.default_rng(seed)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        ids = rng.choice(array.shape[0], size=3, replace=False)
        p0, p1, p2 = array[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-9:
            continue
        normal /= norm
        if preferred is not None and abs(float(np.dot(normal, preferred))) < minimum_alignment:
            continue
        offset = -float(np.dot(normal, p0))
        mask = np.abs(array @ normal + offset) <= distance_threshold
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 3:
        raise ValueError("RANSAC could not find a plane")
    fitted = fit_plane_svd(array[best_mask])
    normal = fitted.normal
    offset = fitted.offset
    if preferred is not None and float(np.dot(normal, preferred)) < 0.0:
        normal = -normal
        offset = -offset
    return PlaneFit(
        normal=normal,
        offset=offset,
        distance=fitted.distance,
        rms_error=fitted.rms_error,
        inlier_count=best_count,
    )


def camera_level_from_floor_normal(normal: Iterable[float]) -> CameraLevel:
    """Estimate optical-frame roll and downward pitch from a floor normal.

    ROS camera optical axes are x-right, y-down, z-forward. A level camera sees
    world-up as [0, -1, 0]. The floor normal is flipped to that hemisphere.
    """

    up = np.asarray(normal, dtype=float).reshape(3)
    norm = float(np.linalg.norm(up))
    if norm <= 1e-12 or not np.isfinite(up).all():
        raise ValueError("normal must be finite and non-zero")
    up = up / norm
    if up[1] > 0.0:
        up = -up
    roll = math.atan2(float(up[0]), float(-up[1]))
    pitch_down = math.atan2(float(up[2]), math.hypot(float(up[0]), float(up[1])))
    return CameraLevel(roll_rad=roll, pitch_down_rad=pitch_down, up_vector=up)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_angle_sequence(angles: Sequence[float]) -> list[float]:
    if not angles:
        return []
    result = [float(angles[0])]
    for angle in angles[1:]:
        result.append(result[-1] + normalize_angle(float(angle) - normalize_angle(result[-1])))
    return result


def linear_odometry_scale(actual_distance: float, reported_distance: float) -> float:
    if not math.isfinite(actual_distance) or actual_distance <= 0.0:
        raise ValueError("actual_distance must be finite and positive")
    if not math.isfinite(reported_distance) or abs(reported_distance) <= 1e-9:
        raise ValueError("reported_distance must be finite and non-zero")
    return actual_distance / abs(reported_distance)


def rotational_odometry_scale(actual_angle: float, reported_angle: float) -> float:
    if not math.isfinite(actual_angle) or actual_angle <= 0.0:
        raise ValueError("actual_angle must be finite and positive")
    if not math.isfinite(reported_angle) or abs(reported_angle) <= 1e-9:
        raise ValueError("reported_angle must be finite and non-zero")
    return actual_angle / abs(reported_angle)


def fit_line_2d(points: Iterable[Iterable[float]]) -> LineFit2D:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("points must be an N x 2 array")
    array = array[np.isfinite(array).all(axis=1)]
    if array.shape[0] < 2:
        raise ValueError("at least two finite points are required")
    centroid = np.mean(array, axis=0)
    _, _, vh = np.linalg.svd(array - centroid, full_matrices=False)
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]], dtype=float)
    normal /= np.linalg.norm(normal)
    if float(np.dot(normal, centroid)) < 0.0:
        normal = -normal
    distance = float(np.dot(normal, centroid))
    offset = -distance
    residuals = array @ normal + offset
    return LineFit2D(
        normal=normal,
        offset=offset,
        distance=distance,
        normal_angle=math.atan2(float(normal[1]), float(normal[0])),
        rms_error=float(np.sqrt(np.mean(np.square(residuals)))),
        point_count=int(array.shape[0]),
    )


def fit_board_from_scan(
    *,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    expected_angle: float,
    half_width: float,
    minimum_range: float,
    maximum_range: float,
    range_tolerance: float = 0.30,
) -> LineFit2D:
    if angle_increment == 0.0:
        raise ValueError("angle_increment must be non-zero")
    candidates: list[tuple[float, float]] = []
    for index, raw in enumerate(ranges):
        value = float(raw)
        angle = angle_min + index * angle_increment
        if abs(normalize_angle(angle - expected_angle)) > half_width:
            continue
        if math.isfinite(value) and minimum_range <= value <= maximum_range:
            candidates.append((angle, value))
    if len(candidates) < 4:
        raise ValueError("not enough valid board points in the requested sector")
    median_range = float(np.median([value for _, value in candidates]))
    selected = [
        (value * math.cos(angle), value * math.sin(angle))
        for angle, value in candidates
        if abs(value - median_range) <= range_tolerance
    ]
    if len(selected) < 4:
        raise ValueError("board range filtering left too few points")
    fit = fit_line_2d(selected)
    # A front board can generate the equivalent opposite normal. Pick the
    # representation closest to the expected direction without changing the line.
    if abs(normalize_angle(fit.normal_angle - expected_angle)) > math.pi / 2.0:
        normal = -fit.normal
        return LineFit2D(
            normal=normal,
            offset=-fit.offset,
            distance=fit.distance,
            normal_angle=normalize_angle(fit.normal_angle + math.pi),
            rms_error=fit.rms_error,
            point_count=fit.point_count,
        )
    return fit


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert a normalized or non-normalized quaternion to roll, pitch, yaw."""

    values = np.asarray([x, y, z, w], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("quaternion must be finite")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = (values / norm).tolist()
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
