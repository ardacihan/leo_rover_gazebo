"""Pure geometry and inverse resampling for occupancy-grid fusion.

ROS ``OccupancyGrid`` origins are poses, not just translations.  Keeping this
module free of ROS imports makes the easy-to-miss origin rotation and map-to-
map transform math directly testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np

Transform = Tuple[float, float, float]


@dataclass(frozen=True)
class GridSpec:
    """A numpy occupancy grid and its complete world pose."""

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])


def transform_points(x, y, transform: Transform):
    """Apply target = R(yaw) * source + translation to scalar/array points."""
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y + tx, s * x + c * y + ty


def inverse_transform_points(x, y, transform: Transform):
    """Map target-frame scalar/array points back into the source frame."""
    tx, ty, yaw = transform
    dx, dy = x - tx, y - ty
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def grid_bounds(spec: GridSpec, transform: Optional[Transform] = None):
    """Axis-aligned bounds after both the grid-origin and inter-map poses."""
    lx = np.asarray([0.0, spec.width * spec.resolution,
                     0.0, spec.width * spec.resolution])
    ly = np.asarray([0.0, 0.0, spec.height * spec.resolution,
                     spec.height * spec.resolution])
    x, y = transform_points(
        lx, ly, (spec.origin_x, spec.origin_y, spec.origin_yaw))
    if transform is not None:
        x, y = transform_points(x, y, transform)
    return float(x.min()), float(y.min()), float(x.max()), float(y.max())


def sample_grid(
    spec: GridSpec,
    shared_x: np.ndarray,
    shared_y: np.ndarray,
    transform: Optional[Transform] = None,
) -> np.ndarray:
    """Inverse-sample ``spec`` at shared-frame cell centres.

    Inverse sampling gives every output pixel one well-defined source value.
    It avoids the diagonal gaps produced when source pixels are forward-splat
    into a rotated output grid.
    """
    x, y = shared_x, shared_y
    if transform is not None:
        x, y = inverse_transform_points(x, y, transform)
    x, y = inverse_transform_points(
        x, y, (spec.origin_x, spec.origin_y, spec.origin_yaw))
    cols = np.floor(x / spec.resolution).astype(np.int64)
    rows = np.floor(y / spec.resolution).astype(np.int64)
    valid = ((cols >= 0) & (cols < spec.width)
             & (rows >= 0) & (rows < spec.height))
    values = np.full(shared_x.shape, -1, dtype=np.int16)
    values[valid] = spec.data[rows[valid], cols[valid]]
    return values


def fuse_grids(
    grids: Iterable[Tuple[GridSpec, Optional[Transform]]],
    occupied_threshold: int = 50,
):
    """Fuse grids into a shared, axis-aligned array.

    Returns ``(data, resolution, min_x, min_y)``. Occupied evidence dominates
    conflicting free evidence, which is the conservative choice for real
    collision planning.
    """
    entries = list(grids)
    if not entries:
        raise ValueError("at least one grid is required")
    resolution = min(spec.resolution for spec, _ in entries)
    bounds = [grid_bounds(spec, tf) for spec, tf in entries]
    min_x = min(bound[0] for bound in bounds)
    min_y = min(bound[1] for bound in bounds)
    max_x = max(bound[2] for bound in bounds)
    max_y = max(bound[3] for bound in bounds)
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))

    rows, cols = np.indices((height, width), dtype=np.float64)
    shared_x = min_x + (cols + 0.5) * resolution
    shared_y = min_y + (rows + 0.5) * resolution
    fused = np.full((height, width), -1, dtype=np.int16)
    for spec, transform in entries:
        incoming = sample_grid(spec, shared_x, shared_y, transform)
        known = incoming >= 0
        empty = fused < 0
        fused[known & empty] = incoming[known & empty]
        conflict = known & ~empty
        occupied = conflict & (
            (incoming >= occupied_threshold) | (fused >= occupied_threshold))
        fused[occupied] = 100
        fused[conflict & ~occupied] = 0
    return fused, resolution, min_x, min_y
