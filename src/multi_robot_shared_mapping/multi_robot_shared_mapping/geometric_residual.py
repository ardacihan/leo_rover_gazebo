"""Undilated occupancy residual and a small residual-minimizing polish.

Confidence can accept a merge that is still visibly shifted. Residual is the
median nearest-occupied distance after the candidate transform, measured on
undilated walls. A lock is refused until that residual is small.
"""

from __future__ import annotations

import math
from typing import Callable, Tuple

import numpy as np

from .grid_registration import cell_centers, invert_transform

GridInfo = Tuple[float, float, float, float]
Transform = Tuple[float, float, float]


def apply_se2(points: np.ndarray, transform: Transform) -> np.ndarray:
    """p_target = R(yaw) p_source + t.  # shape: (N, 2)"""
    if len(points) == 0:
        return points
    dx, dy, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    x = dx + c * points[:, 0] - s * points[:, 1]
    y = dy + s * points[:, 0] + c * points[:, 1]
    return np.column_stack([x, y])


def compose_se2(outer: Transform, inner: Transform) -> Transform:
    """Apply inner first, then outer."""
    ox, oy, oyaw = outer
    ix, iy, iyaw = inner
    c, s = math.cos(oyaw), math.sin(oyaw)
    yaw = math.atan2(math.sin(oyaw + iyaw), math.cos(oyaw + iyaw))
    return (ox + c * ix - s * iy, oy + s * ix + c * iy, yaw)


def _thin(points: np.ndarray, max_points: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    pick = np.random.RandomState(0).choice(len(points), max_points, replace=False)
    return points[pick]


def _nearest_indices(queries: np.ndarray, refs: np.ndarray) -> np.ndarray:
    """Argmin Euclidean neighbor for each query.  # shape: (N,)"""
    n = len(queries)
    idx = np.empty(n, dtype=np.int32)
    chunk = 256
    for i in range(0, n, chunk):
        q = queries[i:i + chunk]
        d2 = ((q[:, None, :] - refs[None, :, :]) ** 2).sum(axis=2)
        idx[i:i + chunk] = d2.argmin(axis=1)
    return idx


def _occupied_points(grid: np.ndarray, info: GridInfo,
                     max_points: int) -> np.ndarray:
    return _thin(cell_centers(grid, info, lambda g: g >= 50), max_points)


def _distances(queries: np.ndarray, refs: np.ndarray) -> np.ndarray:
    idx = _nearest_indices(queries, refs)
    return np.hypot(queries[:, 0] - refs[idx, 0],
                    queries[:, 1] - refs[idx, 1])


def residual_stats(grid1: np.ndarray, info1: GridInfo,
                   grid2: np.ndarray, info2: GridInfo,
                   transform: Transform, max_points: int = 2500,
                   max_pair_m: float = 0.60) -> dict:
    """Residual on the overlapping wall evidence in the stronger direction.

    The robots deliberately explore complementary regions, so a percentile
    over *all* walls measures missing coverage rather than registration.  We
    evaluate both directions, select the one with the larger correspondence
    fraction, and compute residuals only for plausible local wall pairs.  The
    correspondence and undilated-hit fractions still prevent a tiny accidental
    overlap from passing the lock gate.
    """
    empty = {'median_m': float('inf'), 'p90_m': float('inf'),
             'mean_m': float('inf'), 'undilated_hit': 0.0,
             'overlap_fraction': 0.0, 'n_pairs': 0,
             'n_correspondences': 0, 'direction': 'none'}
    p1 = _occupied_points(grid1, info1, max_points)
    p2 = _occupied_points(grid2, info2, max_points)
    if len(p1) < 20 or len(p2) < 20:
        return empty
    forward = _distances(apply_se2(p2, transform), p1)
    reverse = _distances(apply_se2(p1, invert_transform(transform)), p2)
    candidates = []
    for name, dist in (('leo2_to_leo1', forward),
                       ('leo1_to_leo2', reverse)):
        keep = dist <= max_pair_m
        candidates.append((float(keep.mean()), int(keep.sum()), name,
                           dist, dist[keep]))
    _, _, direction, all_dist, dist = max(candidates,
                                          key=lambda item: item[:2])
    if len(dist) < 20:
        return empty
    res = float(info1[2])
    return {
        'median_m': float(np.median(dist)),
        'p90_m': float(np.percentile(dist, 90)),
        'mean_m': float(np.mean(dist)),
        'undilated_hit': float(np.mean(all_dist <= (1.25 * res))),
        'overlap_fraction': float(len(dist) / len(all_dist)),
        'n_pairs': int(len(all_dist)),
        'n_correspondences': int(len(dist)),
        'direction': direction,
    }


def geometric_lock_ok(stats: dict, max_median_m: float = 0.12,
                      max_p90_m: float = 0.30,
                      min_undilated_hit: float = 0.30,
                      min_overlap_fraction: float = 0.55) -> Tuple[bool, str]:
    """True only when the occupancy maps actually overlay, not just score."""
    median = float(stats.get('median_m', float('inf')))
    p90 = float(stats.get('p90_m', float('inf')))
    hit = float(stats.get('undilated_hit', 0.0))
    overlap = float(stats.get('overlap_fraction', 0.0))
    if overlap < min_overlap_fraction:
        return False, (f"wall correspondence {overlap:.3f} < "
                       f"{min_overlap_fraction:.2f}")
    if median > max_median_m:
        return False, f"median occupancy residual {median:.3f} m > {max_median_m:.2f} m"
    if p90 > max_p90_m:
        return False, f"p90 occupancy residual {p90:.3f} m > {max_p90_m:.2f} m"
    if hit < min_undilated_hit:
        return False, f"undilated wall hit {hit:.3f} < {min_undilated_hit:.2f}"
    return True, "geometry aligned"


def _kabsch_2d(src: np.ndarray, dst: np.ndarray) -> Transform:
    """SE(2) taking src onto dst."""
    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    h = (src - cs).T @ (dst - cd)
    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt = vt.copy()
        vt[-1] *= -1.0
        rot = vt.T @ u.T
    t = cd - rot @ cs
    return (float(t[0]), float(t[1]), float(math.atan2(rot[1, 0], rot[0, 0])))


def residual_polish(grid1: np.ndarray, info1: GridInfo,
                    grid2: np.ndarray, info2: GridInfo, seed: Transform,
                    iterations: int = 6,
                    max_pair_m: float = 0.60,
                    max_points: int = 1800) -> Tuple[Transform, dict]:
    """Close a leftover raster/SLAM offset by nearest-wall correspondences."""
    p1 = _occupied_points(grid1, info1, max_points)
    p2 = _occupied_points(grid2, info2, max_points)
    current = tuple(float(v) for v in seed)
    if len(p1) < 40 or len(p2) < 40:
        return current, residual_stats(grid1, info1, grid2, info2, current)
    for _ in range(iterations):
        q = apply_se2(p2, current)
        dest = p1[_nearest_indices(q, p1)]
        dist = np.hypot(q[:, 0] - dest[:, 0], q[:, 1] - dest[:, 1])
        keep = dist < max_pair_m
        if int(keep.sum()) < 40:
            break
        current = compose_se2(_kabsch_2d(q[keep], dest[keep]), current)
    stats = residual_stats(grid1, info1, grid2, info2, current, max_points)
    stats['polish_translation_m'] = math.hypot(
        current[0] - seed[0], current[1] - seed[1])
    stats['polish_yaw_deg'] = abs(math.degrees(math.atan2(
        math.sin(current[2] - seed[2]), math.cos(current[2] - seed[2]))))
    return current, stats
