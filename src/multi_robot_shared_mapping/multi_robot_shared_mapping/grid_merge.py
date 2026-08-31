"""Pose-aware occupancy-grid resampling and fusion (no ROS dependencies)."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .grid_registration import invert_transform

GridInfo = Tuple[float, float, float, float]
Transform = Tuple[float, float, float]


def _apply(x, y, transform: Optional[Transform]):
    if transform is None:
        return x, y
    dx, dy, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return dx + c * x - s * y, dy + s * x + c * y


def grid_bounds(shape, info: GridInfo,
                transform: Optional[Transform] = None):
    h, w = shape
    ox, oy, res, yaw = info
    local = ((0.0, 0.0), (w * res, 0.0),
             (0.0, h * res), (w * res, h * res))
    c, s = math.cos(yaw), math.sin(yaw)
    points = []
    for lx, ly in local:
        x, y = ox + c * lx - s * ly, oy + s * lx + c * ly
        points.append(_apply(x, y, transform))
    xs, ys = zip(*points)
    return min(xs), min(ys), max(xs), max(ys)


def _sample(grid, info: GridInfo, x, y,
            transform: Optional[Transform]):
    if transform is not None:
        x, y = _apply(x, y, invert_transform(transform))
    ox, oy, res, yaw = info
    dx, dy = x - ox, y - oy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    cols = np.floor(lx / res).astype(int)
    rows = np.floor(ly / res).astype(int)
    valid = ((cols >= 0) & (cols < grid.shape[1]) &
             (rows >= 0) & (rows < grid.shape[0]))
    out = np.full(x.shape, -1, dtype=np.int16)
    out[valid] = grid[rows[valid], cols[valid]]
    return out


def merge_grids(entries: Sequence[Tuple[np.ndarray, GridInfo,
                                        Optional[Transform]]],
                occupied_threshold: int = 50):
    """Resample every grid at output cell centers and fuse occupied-wins.

    Output-centric inverse sampling avoids holes under rotation and correctly
    handles different input resolutions and rotated occupancy-grid origins.
    """
    if not entries:
        return None, None
    resolution = min(float(info[2]) for _, info, _ in entries)
    bounds = [grid_bounds(grid.shape, info, transform)
              for grid, info, transform in entries]
    min_x = min(b[0] for b in bounds)
    min_y = min(b[1] for b in bounds)
    max_x = max(b[2] for b in bounds)
    max_y = max(b[3] for b in bounds)
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    xs = min_x + (np.arange(width) + 0.5) * resolution
    ys = min_y + (np.arange(height) + 0.5) * resolution
    x, y = np.meshgrid(xs, ys)
    merged = np.full((height, width), -1, dtype=np.int16)
    for grid, info, transform in entries:
        incoming = _sample(grid, info, x, y, transform)
        known = incoming >= 0
        empty = merged < 0
        merged[known & empty] = incoming[known & empty]
        occupied = known & (incoming >= occupied_threshold)
        merged[occupied] = 100
        free = known & ~occupied & (merged < occupied_threshold)
        merged[free] = 0
    return merged, (min_x, min_y, resolution, 0.0)
