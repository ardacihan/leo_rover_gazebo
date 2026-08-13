#!/usr/bin/env python3

"""Pure coverage/frontier helpers for exploration. No ROS imports.

Exploration needs an answer to "where is there no data yet". A frontier is a
known-free cell touching unknown space, so the frontiers are exactly the places
worth driving to.
"""

import numpy as np

UNKNOWN = -1


def coverage_stats(grid, occupied_threshold=65):
    """Return (unknown, free, occupied) cell fractions of an OccupancyGrid array."""
    cells = np.asarray(grid)
    total = cells.size
    if total == 0:
        return 0.0, 0.0, 0.0
    unknown = int(np.count_nonzero(cells == UNKNOWN))
    occupied = int(np.count_nonzero(cells >= occupied_threshold))
    free = total - unknown - occupied
    return unknown / total, free / total, occupied / total


def frontier_mask(cells, free_threshold=25, occupied_threshold=65):
    """Mark known-free cells that touch unknown space in 4-connectivity.

    Cells adjacent to an obstacle are excluded: the far side of a wall is
    unknown but unreachable, and treating it as a frontier is what makes naive
    explorers drive at walls forever.
    """
    grid = np.asarray(cells)
    free = (grid >= 0) & (grid <= free_threshold)
    unknown = grid == UNKNOWN
    blocked = grid >= occupied_threshold

    def shift(mask, dy, dx):
        out = np.zeros_like(mask)
        height, width = mask.shape
        ys = slice(max(0, dy), height + min(0, dy))
        xs = slice(max(0, dx), width + min(0, dx))
        yd = slice(max(0, -dy), height + min(0, -dy))
        xd = slice(max(0, -dx), width + min(0, -dx))
        out[yd, xd] = mask[ys, xs]
        return out

    touches_unknown = np.zeros_like(free)
    touches_blocked = np.zeros_like(free)
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        touches_unknown |= shift(unknown, dy, dx)
        touches_blocked |= shift(blocked, dy, dx)
    return free & touches_unknown & ~touches_blocked


def cluster_frontiers(mask, min_cells=4):
    """Group frontier cells into 8-connected clusters, largest first.

    Returns a list of (cell_count, mean_row, mean_col). Clusters below
    `min_cells` are dropped: single stray cells are usually scan noise at a
    map edge, and chasing them wastes a run.
    """
    mask = np.asarray(mask)
    seen = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    clusters = []
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or seen[y, x]:
                continue
            stack = [(y, x)]
            seen[y, x] = True
            cells = []
            while stack:
                cy, cx = stack.pop()
                cells.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if (0 <= ny < height and 0 <= nx < width
                                and mask[ny, nx] and not seen[ny, nx]):
                            seen[ny, nx] = True
                            stack.append((ny, nx))
            if len(cells) >= min_cells:
                rows = [c[0] for c in cells]
                cols = [c[1] for c in cells]
                clusters.append((len(cells),
                                 sum(rows) / len(rows),
                                 sum(cols) / len(cols)))
    clusters.sort(reverse=True)
    return clusters


def cluster_to_map_xy(row, col, origin_x, origin_y, resolution):
    """Convert a cluster centroid cell to map-frame metres."""
    return (origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution)
