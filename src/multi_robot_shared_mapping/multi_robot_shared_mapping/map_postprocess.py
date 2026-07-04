#!/usr/bin/env python3
"""
Occupancy-grid cleaning for /shared_map_cleaned (no ROS imports).

Visualization-oriented cleanup only; alignment decisions always use the raw
merged map so cleaning can never hide alignment errors:
- speckle removal: isolated occupied cells with no occupied neighbor are
  almost always sensor noise or double-scan artifacts, not walls.
- unknown-island fill: unknown cells fully surrounded by free space are
  rendered as free for a cleaner look.
"""

from __future__ import annotations

import numpy as np


def _neighbor_count(mask: np.ndarray) -> np.ndarray:
    """8-connected neighbor count for a boolean mask.  # shape: (H, W)"""
    padded = np.pad(mask.astype(np.int16), 1)
    count = np.zeros_like(mask, dtype=np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            count += padded[1 + dy:1 + dy + mask.shape[0],
                            1 + dx:1 + dx + mask.shape[1]]
    return count


def clean_occupancy_grid(
    grid: np.ndarray,
    occupied_threshold: int = 50,
    min_occupied_neighbors: int = 1,
) -> np.ndarray:
    """
    Return a cleaned copy of an occupancy grid (-1 unknown, 0 free, 100 occ).

    Occupied cells with fewer than min_occupied_neighbors occupied neighbors
    become free; unknown cells fully surrounded by free become free.
    """
    cleaned = grid.copy()
    occupied = cleaned >= occupied_threshold
    free = (cleaned >= 0) & ~occupied
    unknown = cleaned < 0

    occ_neighbors = _neighbor_count(occupied)
    speckles = occupied & (occ_neighbors < min_occupied_neighbors)
    cleaned[speckles] = 0

    free_neighbors = _neighbor_count(free)
    isolated_unknown = unknown & (free_neighbors >= 8)
    cleaned[isolated_unknown] = 0

    return cleaned
