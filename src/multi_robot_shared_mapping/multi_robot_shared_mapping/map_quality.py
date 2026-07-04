#!/usr/bin/env python3
"""
Local SLAM map quality guard + minimal submap metadata (no ROS imports).

Detects local map corruption (e.g. after the robot pushes into a wall) from
the occupancy grid stream itself:
- free -> occupied flips: walls suddenly appearing inside space that was
  already confidently mapped as free (typical smearing signature).
- overall change ratio: a healthy exploring map changes gradually; huge jumps
  between consecutive updates indicate the scan matcher lost track.

Quality is an EMA in [0, 1]; 1.0 = healthy. Submaps are tracked as metadata
only (id + status): a corrupted current "submap" is marked low quality so a
future extension can exclude it from fusion or restart SLAM cleanly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class SubmapRecord:
    submap_id: int
    status: str = "active"  # active | accepted | low_quality


@dataclass
class LocalMapQualityTracker:
    """Per-robot quality estimate updated from consecutive OccupancyGrids."""

    smoothing_alpha: float = 0.3
    flip_ratio_scale: float = 40.0
    change_ratio_scale: float = 10.0
    poor_threshold: float = 0.35
    severe_threshold: float = 0.15

    quality: float = 1.0
    submaps: List[SubmapRecord] = field(default_factory=lambda: [SubmapRecord(0)])
    _prev: Optional[Tuple[np.ndarray, float, float, float]] = None  # grid, ox, oy, res

    def update(
        self,
        data,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        occupied_threshold: int = 50,
    ) -> float:
        """Feed the newest map; returns the smoothed quality in [0, 1]."""
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)

        if self._prev is not None:
            flip_ratio, change_ratio = self._compare(grid, resolution, origin_x, origin_y,
                                                     occupied_threshold)
            instant = 1.0 / (
                1.0
                + self.flip_ratio_scale * flip_ratio
                + self.change_ratio_scale * change_ratio
            )
            self.quality = (
                (1.0 - self.smoothing_alpha) * self.quality
                + self.smoothing_alpha * instant
            )
            self._update_submap_status()

        self._prev = (grid, origin_x, origin_y, resolution)
        return self.quality

    def _compare(self, grid, resolution, origin_x, origin_y, occupied_threshold):
        """Overlap-aligned comparison against the previous map snapshot."""
        prev_grid, prev_ox, prev_oy, prev_res = self._prev
        if abs(prev_res - resolution) > 1e-6:
            return 0.0, 0.0  # resolution changed; skip comparison

        # Cell offset of the new grid origin inside the previous grid.
        off_x = int(round((origin_x - prev_ox) / resolution))
        off_y = int(round((origin_y - prev_oy) / resolution))

        # Overlapping window in previous-grid coordinates.
        x0 = max(0, off_x)
        y0 = max(0, off_y)
        x1 = min(prev_grid.shape[1], off_x + grid.shape[1])
        y1 = min(prev_grid.shape[0], off_y + grid.shape[0])
        if x1 <= x0 or y1 <= y0:
            return 0.0, 0.0

        prev_win = prev_grid[y0:y1, x0:x1]
        new_win = grid[y0 - off_y:y1 - off_y, x0 - off_x:x1 - off_x]

        prev_free = (prev_win >= 0) & (prev_win < occupied_threshold)
        new_occupied = new_win >= occupied_threshold
        known_free = int(prev_free.sum())
        if known_free == 0:
            return 0.0, 0.0

        flips = int((prev_free & new_occupied).sum())
        flip_ratio = flips / known_free

        both_known = (prev_win >= 0) & (new_win >= 0)
        known = int(both_known.sum())
        if known == 0:
            return flip_ratio, 0.0
        prev_occ = prev_win >= occupied_threshold
        changed = int((both_known & (prev_occ != new_occupied)).sum())
        return flip_ratio, changed / known

    def _update_submap_status(self):
        current = self.submaps[-1]
        if self.quality < self.poor_threshold and current.status != "low_quality":
            current.status = "low_quality"
        elif self.quality >= self.poor_threshold and current.status == "low_quality":
            # Recovered: retire the corrupted submap and start a new one.
            self.submaps.append(SubmapRecord(current.submap_id + 1))

    @property
    def is_poor(self) -> bool:
        return self.quality < self.poor_threshold

    @property
    def is_severe(self) -> bool:
        return self.quality < self.severe_threshold

    @property
    def current_submap(self) -> SubmapRecord:
        return self.submaps[-1]
