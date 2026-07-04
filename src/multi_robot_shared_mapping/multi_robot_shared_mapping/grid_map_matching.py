#!/usr/bin/env python3
"""
Pure numpy occupancy-grid matching for map-based alignment (no ROS imports).

Pipeline:
1. occupancy_grid_to_points: occupied cells -> world xy points.  # shape: (N, 2)
2. downsample_points: voxel-grid thinning to keep matching fast.
3. build_lookup_grid: rasterize target (leo1) points into a dilated boolean
   grid so a transformed source point "matches" if it lands on/near a target cell.
4. match_maps: coarse-to-fine search over (dx, dy, yaw) candidates, scoring
   how many transformed source (leo2) points hit target cells.

The returned transform maps source (leo2/map) points into target (leo1/map):
target ~= R(yaw) @ source + (dx, dy)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np

Candidate = Tuple[float, float, float, float]  # dx, dy, yaw, score


@dataclass
class GridMatchResult:
    dx: float
    dy: float
    yaw: float
    overlap_score: int            # matched occupied cells (absolute count)
    normalized_overlap_score: float  # matched / source points used, in [0, 1]
    num_source_points: int        # downsampled leo2 occupied points used
    num_target_cells: int         # occupied lookup cells from leo1
    success: bool
    message: str = ""
    free_space_conflict_ratio: float = 0.0  # source walls in target free space
    candidates: List[Candidate] = field(default_factory=list)  # top-K coarse


def occupancy_grid_to_points(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    occupied_threshold: int = 50,
    select: str = "occupied",
) -> np.ndarray:
    """
    Return world xy coordinates of selected cells.  # shape: (N, 2)
    select='occupied': value >= occupied_threshold.
    select='free': 0 <= value < occupied_threshold (known free space).
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    if select == "occupied":
        mask = grid >= occupied_threshold
    else:
        mask = (grid >= 0) & (grid < occupied_threshold)
    iy, ix = np.nonzero(mask)
    xs = origin_x + (ix + 0.5) * resolution
    ys = origin_y + (iy + 0.5) * resolution
    return np.column_stack([xs, ys])


def downsample_points(points: np.ndarray, voxel_size: float, max_points: int = 400) -> np.ndarray:
    """Keep one point per voxel, then stride-subsample down to max_points."""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    thinned = points[np.sort(unique_indices)]
    if len(thinned) > max_points:
        stride = int(math.ceil(len(thinned) / max_points))
        thinned = thinned[::stride]
    return thinned


def build_lookup_grid(
    target_points: np.ndarray,
    resolution: float,
    dilation_cells: int = 1,
    margin_m: float = 1.0,
    free_points: np.ndarray = None,
    free_penalty: int = 1,
) -> Tuple[np.ndarray, float, float]:
    """
    Rasterize target points into a weighted grid:
    exact occupied cell = +2, dilated neighborhood = +1,
    known free space = -free_penalty, unknown = 0.
    Occupied-vs-free conflicts therefore actively lower a candidate's score
    instead of just not helping it, which rejects overlaps that place walls
    inside the other robot's free space.
    """
    min_x = float(target_points[:, 0].min()) - margin_m
    min_y = float(target_points[:, 1].min()) - margin_m
    max_x = float(target_points[:, 0].max()) + margin_m
    max_y = float(target_points[:, 1].max()) + margin_m

    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    exact = np.zeros((height, width), dtype=bool)

    ix = np.floor((target_points[:, 0] - min_x) / resolution).astype(int)
    iy = np.floor((target_points[:, 1] - min_y) / resolution).astype(int)
    exact[iy, ix] = True

    dilated = exact.copy()
    if dilation_cells > 0:
        # Margin guarantees border cells are empty, so np.roll wrap is harmless.
        for shift_y in range(-dilation_cells, dilation_cells + 1):
            for shift_x in range(-dilation_cells, dilation_cells + 1):
                dilated |= np.roll(np.roll(exact, shift_y, axis=0), shift_x, axis=1)

    weights = dilated.astype(np.int16) + exact.astype(np.int16)

    if free_points is not None and len(free_points) > 0:
        fx = np.floor((free_points[:, 0] - min_x) / resolution).astype(int)
        fy = np.floor((free_points[:, 1] - min_y) / resolution).astype(int)
        in_bounds = (fx >= 0) & (fx < width) & (fy >= 0) & (fy < height)
        free_mask = np.zeros((height, width), dtype=bool)
        free_mask[fy[in_bounds], fx[in_bounds]] = True
        # Free space never overrides occupied/dilated cells.
        weights[free_mask & ~dilated] = -abs(free_penalty)

    return weights, min_x, min_y


def _score_translations(
    rotated_points: np.ndarray,
    dxs: np.ndarray,
    dys: np.ndarray,
    lookup: np.ndarray,
    min_x: float,
    min_y: float,
    resolution: float,
) -> np.ndarray:
    """Weighted score for every (dx, dy) candidate at once.  # shape: (M,)"""
    ix = np.floor((rotated_points[:, 0][None, :] + dxs[:, None] - min_x) / resolution).astype(int)
    iy = np.floor((rotated_points[:, 1][None, :] + dys[:, None] - min_y) / resolution).astype(int)

    in_bounds = (
        (ix >= 0) & (ix < lookup.shape[1]) & (iy >= 0) & (iy < lookup.shape[0])
    )
    ix_clipped = np.clip(ix, 0, lookup.shape[1] - 1)
    iy_clipped = np.clip(iy, 0, lookup.shape[0] - 1)
    values = lookup[iy_clipped, ix_clipped] * in_bounds
    return values.sum(axis=1)


def count_matched_points(
    source_points: np.ndarray,
    dx: float,
    dy: float,
    yaw: float,
    lookup: np.ndarray,
    min_x: float,
    min_y: float,
    resolution: float,
) -> Tuple[int, int]:
    """
    Evaluate transformed source points against the lookup grid.
    Returns (matched, conflicting): points on/near target walls vs points
    landing in target known free space.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = source_points @ np.array([[c, s], [-s, c]])
    ix = np.floor((rotated[:, 0] + dx - min_x) / resolution).astype(int)
    iy = np.floor((rotated[:, 1] + dy - min_y) / resolution).astype(int)
    in_bounds = (
        (ix >= 0) & (ix < lookup.shape[1]) & (iy >= 0) & (iy < lookup.shape[0])
    )
    ix_c = np.clip(ix, 0, lookup.shape[1] - 1)
    iy_c = np.clip(iy, 0, lookup.shape[0] - 1)
    values = lookup[iy_c, ix_c]
    matched = int(((values > 0) & in_bounds).sum())
    conflicting = int(((values < 0) & in_bounds).sum())
    return matched, conflicting


def _search(
    source_points: np.ndarray,
    lookup: np.ndarray,
    min_x: float,
    min_y: float,
    resolution: float,
    center: Tuple[float, float, float],
    xy_range: float,
    yaw_range: float,
    xy_step: float,
    yaw_step: float,
) -> Tuple[Tuple[float, float, float, int], List[Candidate]]:
    """
    Exhaustive search over the candidate grid.
    Returns (best (dx, dy, yaw, score), raw candidate list) where candidates
    are the top few translations per yaw (used for ambiguity detection).
    """
    cx, cy, cyaw = center
    dx_values = np.arange(cx - xy_range, cx + xy_range + xy_step * 0.5, xy_step)
    dy_values = np.arange(cy - xy_range, cy + xy_range + xy_step * 0.5, xy_step)
    yaw_values = np.arange(cyaw - yaw_range, cyaw + yaw_range + yaw_step * 0.5, yaw_step)

    dx_grid, dy_grid = np.meshgrid(dx_values, dy_values)
    dxs = dx_grid.ravel()
    dys = dy_grid.ravel()

    # Tiny penalty (< 1 hit) breaks ties between equally scoring candidates
    # in favor of the one closest to the search center.
    tie_break = 1e-3 * np.hypot(dxs - cx, dys - cy)

    best = (cx, cy, cyaw, -1, -math.inf)
    candidates: List[Candidate] = []
    per_yaw_keep = min(3, len(dxs))
    for yaw in yaw_values:
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = source_points @ np.array([[c, s], [-s, c]])
        scores = _score_translations(rotated, dxs, dys, lookup, min_x, min_y, resolution)
        penalized = scores - tie_break - 1e-3 * abs(yaw - cyaw)

        top_idx = np.argpartition(penalized, -per_yaw_keep)[-per_yaw_keep:]
        for idx in top_idx:
            candidates.append(
                (float(dxs[idx]), float(dys[idx]), float(yaw), float(scores[idx]))
            )

        idx = int(penalized.argmax())
        if penalized[idx] > best[4]:
            best = (
                float(dxs[idx]), float(dys[idx]), float(yaw),
                int(scores[idx]), float(penalized[idx]),
            )
    return best[:4], candidates


def match_maps(
    source_points: np.ndarray,
    target_points: np.ndarray,
    *,
    match_resolution: float = 0.15,
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    xy_range: float = 15.0,
    yaw_range: float = math.pi,
    coarse_xy_step: float = 0.75,
    coarse_yaw_step: float = math.radians(15.0),
    fine_iterations: int = 2,
    target_free_points: np.ndarray = None,
) -> GridMatchResult:
    """
    Coarse-to-fine grid matching.

    source_points: downsampled leo2 occupied points in leo2/map.  # shape: (N, 2)
    target_points: downsampled leo1 occupied points in leo1/map.  # shape: (M, 2)
    target_free_points: optional leo1 free-space points; candidates placing
    leo2 walls inside leo1 free space are penalized.
    center / xy_range / yaw_range define the search window (hybrid mode
    passes the tag estimate as center with a small range).
    """
    if len(source_points) == 0 or len(target_points) == 0:
        return GridMatchResult(
            dx=0.0, dy=0.0, yaw=0.0,
            overlap_score=0, normalized_overlap_score=0.0,
            num_source_points=len(source_points),
            num_target_cells=len(target_points),
            success=False, message="No occupied points in one of the maps",
        )

    lookup, min_x, min_y = build_lookup_grid(
        target_points, match_resolution, free_points=target_free_points
    )

    (dx, dy, yaw, _), raw_candidates = _search(
        source_points, lookup, min_x, min_y, match_resolution,
        center, xy_range, yaw_range, coarse_xy_step, coarse_yaw_step,
    )

    xy_step = coarse_xy_step
    yaw_step = coarse_yaw_step
    for _ in range(fine_iterations):
        # Shrink the window to the previous step size and refine.
        new_xy_step = max(xy_step / 5.0, 0.05)
        new_yaw_step = max(yaw_step / 5.0, math.radians(1.0))
        (dx, dy, yaw, _), _ = _search(
            source_points, lookup, min_x, min_y, match_resolution,
            (dx, dy, yaw), xy_step, yaw_step, new_xy_step, new_yaw_step,
        )
        xy_step = new_xy_step
        yaw_step = new_yaw_step

    matched, conflicting = count_matched_points(
        source_points, dx, dy, yaw, lookup, min_x, min_y, match_resolution
    )
    normalized = matched / float(len(source_points))
    return GridMatchResult(
        dx=dx, dy=dy, yaw=_normalize_angle(yaw),
        overlap_score=matched,
        normalized_overlap_score=normalized,
        num_source_points=len(source_points),
        num_target_cells=int((lookup >= 2).sum()),
        success=True,
        message="Match computed",
        free_space_conflict_ratio=conflicting / float(len(source_points)),
        candidates=raw_candidates,
    )


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
