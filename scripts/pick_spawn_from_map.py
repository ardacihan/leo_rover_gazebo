#!/usr/bin/env python3
"""Pick a well-clear spawn from a SLAM map a rover actually built.

husarion_office uses mesh collisions, so `scripts/world_ground_truth.py` cannot
rasterize it and the spawn-clearance check that works for the authored worlds
is useless there. The authored leo2 spawn (2.36, -11.27) put the rover
nose-to-a-wall: it moved 1.3 m in 25 minutes, mapped 16.7 m^2, and its camera
saw a blank wall for 300+ frames.

A map one rover genuinely drove is better evidence than a guess. This scores
every free cell by its distance to the nearest occupied/unknown cell and
reports the most open candidates, optionally constrained to a region and to a
minimum separation from the other rover's spawn (they must start in different
rooms, not merely different cells).

Usage:
  pick_spawn_from_map.py <map_stem> [--region xmin xmax ymin ymax]
                         [--away-from X Y --min-sep M] [--top N]
"""
import argparse
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_multirobot_media import read_map   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_stem')
    ap.add_argument('--region', nargs=4, type=float, default=None,
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    ap.add_argument('--away-from', nargs=2, type=float, default=None,
                    metavar=('X', 'Y'))
    ap.add_argument('--min-sep', type=float, default=8.0)
    ap.add_argument('--top', type=int, default=6)
    args = ap.parse_args()

    cls, extent = read_map(args.map_stem)
    if cls is None:
        print(f'no map at {args.map_stem}'); return 1
    h, w = cls.shape
    res = (extent[1] - extent[0]) / w
    ox, oy = extent[0], extent[2]

    # Free only. Unknown counts as blocked: a rover must not spawn where the
    # map cannot vouch for the space.
    free = (cls == 0)
    dist = ndimage.distance_transform_edt(free) * res

    ys, xs = np.mgrid[0:h, 0:w]
    wx = ox + (xs + 0.5) * res
    wy = oy + (ys + 0.5) * res

    mask = free.copy()
    if args.region:
        xmin, xmax, ymin, ymax = args.region
        mask &= (wx >= xmin) & (wx <= xmax) & (wy >= ymin) & (wy <= ymax)
    if args.away_from:
        ax_, ay_ = args.away_from
        mask &= (np.hypot(wx - ax_, wy - ay_) >= args.min_sep)

    scored = dist * mask
    if not scored.any():
        print('no candidate cells match the constraints'); return 1

    flat = np.argsort(scored.ravel())[::-1][:args.top * 400]
    picked = []
    for idx in flat:
        r, c = divmod(int(idx), w)
        if scored[r, c] <= 0:
            break
        x, y = float(wx[r, c]), float(wy[r, c])
        # Spread the candidates out so they are genuinely different places.
        if any(np.hypot(x - px, y - py) < 2.0 for px, py, _ in picked):
            continue
        picked.append((x, y, float(dist[r, c])))
        if len(picked) >= args.top:
            break

    print(f'{args.map_stem}: {free.sum()} free cells, res {res:.3f} m')
    for x, y, d in picked:
        print(f'  ({x:7.2f}, {y:7.2f})  clearance {d:.2f} m'
              + (f'  sep {np.hypot(x - args.away_from[0], y - args.away_from[1]):.1f} m'
                 if args.away_from else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
