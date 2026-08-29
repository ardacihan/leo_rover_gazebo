#!/usr/bin/env python3
"""Render both rovers' local maps in leo1's frame, as one picture.

When the aligner abstains -- correctly, if the rovers never built enough
common evidence -- the live /shared_map holds only leo1, and merged_map.png
understates what the pair actually covered. The scoring path in
phase2_metrics.py already unions the two recorded local maps under the
authored spawn offset; this draws that same union so it can be looked at.

The transform is authored ground truth, used for SCORING ONLY. It is never
published to either explorer, exactly as in phase2_metrics.py. A figure made
this way must be captioned as an offline evaluation view, not as a map the
system produced online.

    python3 scripts/render_union_map.py <world> <run_dir> [out.png]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'leo_rover_gazebo', 'launch'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib                                        # noqa: E402
matplotlib.use('Agg')                                    # noqa: E402
import matplotlib.pyplot as plt                          # noqa: E402
from phase2_metrics import load_map                      # noqa: E402
from spawn_poses import bounds_in_robot_frame, relative_offset  # noqa: E402

UNKNOWN = -1


def union(run_dir, world, res=0.05):
    """Rasterise both local maps into leo1's frame.

    Deliberately mirrors phase2_metrics.truth_union_area: the same world-bounds
    canvas and the same np.floor indexing, so the picture and the scored number
    describe the same cells. (Sizing the canvas off the maps' own extents and
    indexing with astype(int) does not: the truncation is not injective once
    the canvas origin is offset, and cells silently collapse onto each other.)
    """
    g1, i1 = load_map(os.path.join(run_dir, 'leo1_map'))
    g2, i2 = load_map(os.path.join(run_dir, 'leo2_map'))
    if g1 is None or g2 is None:
        raise SystemExit(f'{run_dir}: needs both leo1_map and leo2_map')

    layers = [(g1, i1, (0.0, 0.0, 0.0)),
              (g2, i2, relative_offset(world))]
    bounds = bounds_in_robot_frame(world, 'leo1')
    if bounds is None:
        raise SystemExit(f'unknown world {world!r}')
    xmin, xmax, ymin, ymax = bounds
    w = max(1, int(math.ceil((xmax - xmin) / res)))
    h = max(1, int(math.ceil((ymax - ymin) / res)))
    out = np.full((h, w), UNKNOWN, dtype=np.int16)

    for grid, info, (tx, ty, yaw) in layers:
        rows, cols = np.nonzero(grid != UNKNOWN)
        if not rows.size:
            continue
        x = info[0] + (cols + 0.5) * info[2]
        y = info[1] + (rows + 0.5) * info[2]
        c, s = math.cos(yaw), math.sin(yaw)
        ci = np.floor((tx + c * x - s * y - xmin) / res).astype(int)
        ri = np.floor((ty + s * x + c * y - ymin) / res).astype(int)
        ok = (ci >= 0) & (ci < w) & (ri >= 0) & (ri < h)
        # occupied wins over free, so walls survive the overlay
        cur = out[ri[ok], ci[ok]]
        new = grid[rows, cols][ok].astype(np.int16)
        out[ri[ok], ci[ok]] = np.where(new > cur, new, cur)
    return out, (xmin, ymin, res)


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        raise SystemExit(2)
    world, run_dir = sys.argv[1], sys.argv[2]
    out_png = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        run_dir, 'union_map.png')
    grid, (ox, oy, res) = union(run_dir, world)

    known = grid != UNKNOWN
    occ = grid >= 65
    img = np.full(grid.shape, 0.85)      # unknown
    img[known] = 0.97                    # free
    img[occ] = 0.0                       # walls

    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=[ox, ox + grid.shape[1] * res,
                      oy, oy + grid.shape[0] * res],
              interpolation='nearest')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{os.path.basename(run_dir.rstrip("/"))} - both local maps '
                 f'in leo1 frame\n{known.sum() * res * res:.1f} m2 known, '
                 f'{occ.sum() * res * res:.1f} m2 occupied '
                 f'(offline scoring view, authored transform)')
    ax.grid(alpha=0.25, lw=0.4)
    fig.tight_layout()
    fig.savefig(out_png)
    print(f'wrote {out_png}: {known.sum() * res * res:.1f} m2 known')


if __name__ == '__main__':
    main()
