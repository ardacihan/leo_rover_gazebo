#!/usr/bin/env python3
"""Fuse two saved per-robot maps under a given leo2->leo1 transform.

Iterating alignment by re-simulating costs 25 minutes a go; iterating it on two
saved .pgm files costs about a second, which is why the maps are always saved
separately. This also rebuilds a merged map when the live one could not be
captured -- `shared_map_merger` publishes /shared_map VOLATILE and
`map_saver_cli` only ever subscribes TRANSIENT_LOCAL, so the live merged map is
unsaveable with the stock tool.

Fusion is max-occupancy over the union, in leo1's frame: a cell is occupied if
either rover saw it occupied, free if either saw it free, unknown otherwise.
That deliberately does NOT blend -- blending is what turns two walls 30 cm
apart into one fat grey smear and hides a bad alignment.

Usage:
  fuse_maps_offline.py <leo1_stem> <leo2_stem> <out_stem> --tf X Y YAW_DEG
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_multirobot_media import read_map   # noqa: E402

FREE, UNKNOWN, OCC = 0, 1, 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('leo1_stem'); ap.add_argument('leo2_stem')
    ap.add_argument('out_stem')
    ap.add_argument('--tf', nargs=3, type=float, required=True,
                    metavar=('X', 'Y', 'YAW_DEG'),
                    help='pose of leo2/map expressed in leo1/map')
    ap.add_argument('--res', type=float, default=0.05)
    args = ap.parse_args()

    a, ea = read_map(args.leo1_stem)
    b, eb = read_map(args.leo2_stem)
    if a is None or b is None:
        print('missing input map'); return 1
    res = args.res
    tx, ty, tyaw = args.tf[0], args.tf[1], np.radians(args.tf[2])

    # World-coordinate corners of leo2's grid after the transform.
    hb, wb = b.shape
    ys, xs = np.mgrid[0:hb, 0:wb]
    bx = eb[0] + (xs + 0.5) * res
    by = eb[2] + (ys + 0.5) * res
    c, s = np.cos(tyaw), np.sin(tyaw)
    bx2 = tx + c * bx - s * by
    by2 = ty + s * bx + c * by

    xmin = min(ea[0], float(bx2.min())); xmax = max(ea[1], float(bx2.max()))
    ymin = min(ea[2], float(by2.min())); ymax = max(ea[3], float(by2.max()))
    W = int(np.ceil((xmax - xmin) / res)); H = int(np.ceil((ymax - ymin) / res))
    out = np.full((H, W), UNKNOWN, dtype=np.uint8)

    def blit(cls, gx, gy):
        cc = np.clip(((gx - xmin) / res).astype(int), 0, W - 1)
        rr = np.clip(((gy - ymin) / res).astype(int), 0, H - 1)
        free = cls == FREE
        occ = cls == OCC
        out[rr[free], cc[free]] = np.where(out[rr[free], cc[free]] == OCC,
                                           OCC, FREE)
        out[rr[occ], cc[occ]] = OCC          # occupied always wins

    ha, wa = a.shape
    ays, axs = np.mgrid[0:ha, 0:wa]
    blit(a, ea[0] + (axs + 0.5) * res, ea[2] + (ays + 0.5) * res)
    blit(b, bx2, by2)

    img = np.full(out.shape, 205, dtype=np.uint8)
    img[out == FREE] = 254
    img[out == OCC] = 0
    with open(args.out_stem + '.pgm', 'wb') as fh:
        fh.write(b'P5\n'); fh.write(f'{W} {H}\n255\n'.encode())
        fh.write(np.flipud(img).tobytes())
    with open(args.out_stem + '.yaml', 'w') as fh:
        fh.write(f'image: {os.path.basename(args.out_stem)}.pgm\n'
                 f'resolution: {res}\norigin: [{xmin}, {ymin}, 0.0]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    known = int((out != UNKNOWN).sum())
    print(f'{args.out_stem}: {W}x{H} @ {res} m, origin ({xmin:.2f}, {ymin:.2f}), '
          f'{known} known cells = {known * res * res:.1f} m2, '
          f'{int((out == OCC).sum())} occupied')
    return 0


if __name__ == '__main__':
    sys.exit(main())
