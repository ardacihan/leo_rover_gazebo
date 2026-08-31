"""Reachable unexplored room regions still left on an occupancy grid.

A frontier list can go empty while a whole room remains unknown: the doorway
frontier was banned, or clearance ate the last free-unknown edge. Completion
must look at those remaining regions, not only the frontier mask.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


def _label_unknown(unknown: np.ndarray) -> np.ndarray:
    """4-connected labels for unknown cells. 0 is background."""
    try:
        from scipy import ndimage
        labels, _ = ndimage.label(
            unknown, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
        return labels.astype(np.int32)
    except ImportError:
        pass
    h, w = unknown.shape
    labels = np.zeros((h, w), dtype=np.int32)
    next_id = 1
    for r in range(h):
        for c in range(w):
            if not unknown[r, c] or labels[r, c]:
                continue
            labels[r, c] = next_id
            queue = deque([(r, c)])
            while queue:
                y, x = queue.popleft()
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if (0 <= ny < h and 0 <= nx < w
                            and unknown[ny, nx] and not labels[ny, nx]):
                        labels[ny, nx] = next_id
                        queue.append((ny, nx))
            next_id += 1
    return labels


def _touches_free(labels: np.ndarray, free: np.ndarray, region_id: int):
    """True and the free cells adjacent to this unknown component."""
    h, w = labels.shape
    ys, xs = np.nonzero(labels == region_id)
    adjacent = []
    for y, x in zip(ys.tolist(), xs.tolist()):
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx]:
                adjacent.append((ny, nx))
    return adjacent


def remaining_unknown_regions(unknown, free, res, origin_xy,
                              min_area_m2=2.0, max_regions=12,
                              world_bounds=None, bounds_margin_m=0.60):
    """Connected unknown blobs that still touch known free space.

    Returns cluster dicts compatible with frontier_explorer: goal, centroid,
    gain, size_m, area_m2, gr, gc. Goal is the free cell on the doorway
    closest to the unknown centroid, so the rover actually enters the room.
    """
    if unknown.size == 0:
        return []
    labels = _label_unknown(unknown)
    ox, oy = origin_xy
    rooms = []
    for region_id in range(1, int(labels.max()) + 1):
        ys, xs = np.nonzero(labels == region_id)
        area = float(len(ys)) * res * res
        if area < min_area_m2:
            continue
        if world_bounds is not None:
            xmin, xmax, ymin, ymax = world_bounds
            wx = ox + (xs + 0.5) * res
            wy = oy + (ys + 0.5) * res
            # Unknown that lies mostly along the authored world edge is the
            # exterior behind the outer wall, not an indoor room.  A room
            # merely *touching* the edge band must survive: bounds are wall
            # centrelines, so every perimeter room reaches within ~0.1 m of
            # them and an any-cell test silently retired all of those rooms.
            in_band = ((wx <= xmin + bounds_margin_m)
                       | (wx >= xmax - bounds_margin_m)
                       | (wy <= ymin + bounds_margin_m)
                       | (wy >= ymax - bounds_margin_m))
            if float(np.count_nonzero(in_band)) > 0.5 * len(ys):
                continue
        adjacent = _touches_free(labels, free, region_id)
        if not adjacent:
            continue
        cy, cx = float(ys.mean()), float(xs.mean())
        door = min(adjacent, key=lambda p: (p[0] - cy) ** 2 + (p[1] - cx) ** 2)
        # One step toward the room if that cell is still free.
        step = (int(round(np.clip(cy - door[0], -1, 1))),
                int(round(np.clip(cx - door[1], -1, 1))))
        gr, gc = door[0] + step[0], door[1] + step[1]
        if not (0 <= gr < free.shape[0] and 0 <= gc < free.shape[1]
                and free[gr, gc]):
            gr, gc = door
        rooms.append({
            'size_m': math.sqrt(area),
            'area_m2': area,
            'gain': area,
            'centroid': (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
            'goal': (ox + (gc + 0.5) * res, oy + (gr + 0.5) * res),
            'gr': int(gr), 'gc': int(gc),
            'kind': 'remaining_room',
        })
    rooms.sort(key=lambda r: -r['area_m2'])
    return rooms[:max_regions]


def remaining_room_area_m2(rooms):
    return float(sum(r.get('area_m2', 0.0) for r in rooms))
