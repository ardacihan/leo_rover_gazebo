"""Full-resolution local occupancy-grid registration and residual checks.

All transforms use the package contract ``p_target = R(yaw) p_source + t``.
Grid origins are full SE(2) poses, not just lower-left translations.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

GridInfo = Tuple[float, float, float, float]  # origin x/y, resolution, yaw
Transform = Tuple[float, float, float]


def quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def info_from_message(msg) -> GridInfo:
    origin = msg.info.origin
    return (float(origin.position.x), float(origin.position.y),
            float(msg.info.resolution), quaternion_yaw(origin.orientation))


def cell_centers(grid: np.ndarray, info: Sequence[float], mask) -> np.ndarray:
    """Selected cell centers in the occupancy grid's map frame."""
    oyaw = float(info[3]) if len(info) > 3 else 0.0
    ys, xs = np.nonzero(mask(grid))
    if not len(xs):
        return np.zeros((0, 2), dtype=np.float64)
    lx = (xs + 0.5) * float(info[2])
    ly = (ys + 0.5) * float(info[2])
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.column_stack([
        float(info[0]) + c * lx - s * ly,
        float(info[1]) + s * lx + c * ly,
    ])


def invert_transform(transform: Transform) -> Transform:
    dx, dy, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * dx - s * dy, s * dx - c * dy, -yaw)


def _dilate(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    out = mask.copy()
    for _ in range(iterations):
        src = out.copy()
        out[1:, :] |= src[:-1, :]
        out[:-1, :] |= src[1:, :]
        out[:, 1:] |= src[:, :-1]
        out[:, :-1] |= src[:, 1:]
        out[1:, 1:] |= src[:-1, :-1]
        out[:-1, :-1] |= src[1:, 1:]
        out[1:, :-1] |= src[:-1, 1:]
        out[:-1, 1:] |= src[1:, :-1]
    return out


def _hit_evaluator(target_grid: np.ndarray, target_info: GridInfo,
                   source_points: np.ndarray):
    walls = _dilate(target_grid >= 50, 1)
    ox, oy, res, oyaw = target_info
    c0, s0 = math.cos(-oyaw), math.sin(-oyaw)
    h, w = target_grid.shape

    def score(transform: Transform) -> float:
        dx, dy, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        qx = dx + c * source_points[:, 0] - s * source_points[:, 1]
        qy = dy + s * source_points[:, 0] + c * source_points[:, 1]
        rx, ry = qx - ox, qy - oy
        lx = c0 * rx - s0 * ry
        ly = s0 * rx + c0 * ry
        cols = np.floor(lx / res).astype(int)
        rows = np.floor(ly / res).astype(int)
        ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        if not ok.any():
            return 0.0
        return float(walls[rows[ok], cols[ok]].sum()) / len(source_points)

    return score


def registration_hits(grid1: np.ndarray, info1: GridInfo,
                      grid2: np.ndarray, info2: GridInfo,
                      transform: Transform, max_points: int = 6000) -> dict:
    """Wall-hit fractions in both directions for a source-to-target pose."""
    p1 = cell_centers(grid1, info1, lambda g: g >= 50)
    p2 = cell_centers(grid2, info2, lambda g: g >= 50)
    if not len(p1) or not len(p2):
        return {'forward_hit': 0.0, 'reverse_hit': 0.0, 'wall_hit': 0.0}

    def thin(points):
        if len(points) <= max_points:
            return points
        pick = np.random.RandomState(0).choice(
            len(points), max_points, replace=False)
        return points[pick]

    p1, p2 = thin(p1), thin(p2)
    forward = _hit_evaluator(grid1, info1, p2)(transform)
    reverse = _hit_evaluator(grid2, info2, p1)(invert_transform(transform))
    return {'forward_hit': forward, 'reverse_hit': reverse,
            # The smaller map is allowed to validate against the larger map;
            # requiring both directions penalizes complementary exploration.
            'wall_hit': max(forward, reverse)}


def local_refine(grid1: np.ndarray, info1: GridInfo,
                 grid2: np.ndarray, info2: GridInfo, seed: Transform,
                 max_points: int = 6000) -> Tuple[Transform, dict]:
    """Polish a trusted seed and return the full-resolution wall residual.

    The trust region is deliberately local. It can remove raster/SLAM offset,
    but cannot jump to another symmetric office-room mode.
    """
    p1 = cell_centers(grid1, info1, lambda g: g >= 50)
    p2 = cell_centers(grid2, info2, lambda g: g >= 50)
    if len(p1) < 50 or len(p2) < 50:
        return seed, registration_hits(grid1, info1, grid2, info2, seed)
    rng = np.random.RandomState(0)
    if len(p1) > max_points:
        p1 = p1[rng.choice(len(p1), max_points, replace=False)]
    if len(p2) > max_points:
        p2 = p2[rng.choice(len(p2), max_points, replace=False)]

    # Optimize in the direction that maps the smaller wall set into the
    # larger one; this is robust when the rovers explored complementary areas.
    if len(p2) <= len(p1):
        evaluator = _hit_evaluator(grid1, info1, p2)
        score = evaluator
    else:
        evaluator = _hit_evaluator(grid2, info2, p1)
        score = lambda t: evaluator(invert_transform(t))

    best = tuple(float(v) for v in seed)
    best_score = score(best)
    stages = (
        (0.10, math.radians(0.75), 0.70, math.radians(4.0)),
        (0.025, math.radians(0.20), 0.12, math.radians(0.9)),
        (0.010, math.radians(0.05), 0.035, math.radians(0.25)),
    )
    for step_xy, step_yaw, window_xy, window_yaw in stages:
        cx, cy, cyaw = best
        for yaw in np.arange(cyaw - window_yaw,
                             cyaw + window_yaw + step_yaw * 0.5, step_yaw):
            for dx in np.arange(cx - window_xy,
                                cx + window_xy + step_xy * 0.5, step_xy):
                for dy in np.arange(cy - window_xy,
                                    cy + window_xy + step_xy * 0.5, step_xy):
                    candidate = (float(dx), float(dy), float(yaw))
                    value = score(candidate)
                    if value > best_score:
                        best, best_score = candidate, value
    best = (best[0], best[1], math.atan2(math.sin(best[2]), math.cos(best[2])))
    # Dilated wall-hit can accept a 0.2 m ghost. A nearest-wall polish
    # removes that leftover before any lock decision sees the pose.
    from .geometric_residual import residual_polish
    best, residual = residual_polish(grid1, info1, grid2, info2, best)
    metrics = registration_hits(grid1, info1, grid2, info2, best, max_points)
    metrics.update(residual)
    metrics['seed_wall_hit'] = registration_hits(
        grid1, info1, grid2, info2, seed, max_points)['wall_hit']
    metrics['refinement_translation_m'] = math.hypot(
        best[0] - seed[0], best[1] - seed[1])
    metrics['refinement_yaw_deg'] = abs(math.degrees(math.atan2(
        math.sin(best[2] - seed[2]), math.cos(best[2] - seed[2]))))
    return best, metrics
