#!/usr/bin/env python3
"""Generate ArUco marker textures for the Gazebo worlds.

The office/depot worlds already carry small `marker_N` boxes as flat magenta
placeholders for the mock detector. They cannot be *seen* by a real detector,
so this writes real DICT_4X4_50 marker images and the world file points its
marker visuals at them through `model://aruco_markers/textures/aruco_<id>.png`.

A quiet zone matters: ArUco needs at least one cell of white border around the
black square or `detectMarkers` will not find the quad at all. The generated
image is the marker plus a 1-cell white margin, and `marker_length` in
`aruco.launch.py` must be the side of the *black square*, not of the image --
which is why `--border-cells` is reported at the end.

Usage:
    python3 scripts/make_aruco_models.py [--ids 1 2 3 ...] [--px 600]
"""

import argparse
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, 'src', 'leo_rover_gazebo', 'models',
                   'aruco_markers', 'textures')

DICT_NAME = 'DICT_4X4_50'
DICT_CELLS = 4          # payload cells per side
BORDER_CELLS = 1        # black border cells the dictionary itself adds
QUIET_CELLS = 1         # white quiet zone this script adds


def draw(mid, px, quiet=True):
    dict_id = getattr(cv2.aruco, DICT_NAME)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dict_id)

    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, mid, px)
    else:
        img = cv2.aruco.drawMarker(dictionary, mid, px)

    if not quiet:
        # No padding: the image *is* the black square, so a plate of side S
        # carries a marker of exactly side S. The quiet zone then has to be
        # real geometry -- a larger white board behind the plate -- which is
        # also how a printed marker actually looks on a wall.
        return img, 0

    # White quiet zone, sized so one dictionary cell = one quiet cell.
    cells = DICT_CELLS + 2 * BORDER_CELLS
    pad = int(round(px / cells * QUIET_CELLS))
    out = np.full((px + 2 * pad, px + 2 * pad), 255, dtype=np.uint8)
    out[pad:pad + px, pad:pad + px] = img
    return out, pad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ids', type=int, nargs='*', default=list(range(1, 9)))
    ap.add_argument('--px', type=int, default=600)
    ap.add_argument('--no-quiet-zone', action='store_true',
                    help='emit the bare black square; the world provides the '
                         'white margin as a backing board')
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    for mid in args.ids:
        img, pad = draw(mid, args.px, quiet=not args.no_quiet_zone)
        suffix = '_bare' if args.no_quiet_zone else ''
        path = os.path.join(OUT, f'aruco_{mid}{suffix}.png')
        cv2.imwrite(path, img)
        print(f'wrote {path}  {img.shape[1]}x{img.shape[0]} px, quiet zone {pad} px')

    total = args.px + 2 * int(round(args.px / (DICT_CELLS + 2 * BORDER_CELLS)))
    ratio = args.px / total
    print(f'\ndictionary   {DICT_NAME}')
    print(f'black square occupies {ratio:.4f} of the image side')
    print('If the world plate is S metres wide, set marker_length = '
          f'{ratio:.4f} * S')


if __name__ == '__main__':
    main()
