#!/usr/bin/env python3
"""Score the marker-free matcher on one saved run pair.

    python3 scripts/_mfm_check_pair.py <run_dir> <world>
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

from merge_benchmark import load_map, wrap_deg      # noqa: E402
from spawn_poses import relative_offset             # noqa: E402
import marker_free_matcher as mfm                   # noqa: E402


def main():
    d, world = sys.argv[1], sys.argv[2]
    g1, i1 = load_map(os.path.join(d, 'leo1_map'))
    g2, i2 = load_map(os.path.join(d, 'leo2_map'))
    if g1 is None or g2 is None:
        print('maps missing in', d)
        return 1
    t = relative_offset(world)

    # every candidate mode, annotated against truth
    modes = mfm.candidate_modes(g1, i1, g2, i2)
    print(f'{len(modes)} candidate modes:')
    for k, (scv, mx, my, mth) in enumerate(modes):
        est, fwd = mfm.polish(g1, i1, g2, i2, mx, my, mth)
        if est is None:
            continue
        rev = mfm.reverse_hit(g1, i1, g2, i2, *est)
        exy = math.hypot(est[0] - t[0], est[1] - t[1])
        eyaw = abs(wrap_deg(math.degrees(est[2] - t[2])))
        tag = 'TRUE' if (exy <= 0.5 and eyaw <= 10) else ''
        print(f'  m{k:02d} coarse={scv:6.3f} q={max(fwd, rev):.3f} '
              f'(f={fwd:.3f} r={rev:.3f}) yaw={math.degrees(est[2]):7.1f} '
              f't=({est[0]:7.2f},{est[1]:7.2f}) err={exy:5.2f}m/{eyaw:6.2f}d {tag}')

    # what does the TRUTH transform itself score?
    est_t, fwd_t = mfm.polish(g1, i1, g2, i2, t[0], t[1], t[2])
    if est_t is not None:
        rev_t = mfm.reverse_hit(g1, i1, g2, i2, *est_t)
        exy = math.hypot(est_t[0] - t[0], est_t[1] - t[1])
        print(f'TRUTH-seeded polish: q={max(fwd_t, rev_t):.3f} '
              f'(f={fwd_t:.3f} r={rev_t:.3f}) drifted {exy:.2f} m from spawn truth')

    est, diag = mfm.match_diag(g1, i1, g2, i2)
    print('decision:', 'ABSTAIN' if est is None else est, diag['reason'],
          f"best={diag['best_hit']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
