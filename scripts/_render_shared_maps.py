#!/usr/bin/env python3
"""Render the per-rover shared maps of a distributed run: _render_shared_maps.py <run_dir>"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt              # noqa: E402
from render_multirobot_media import read_pgm  # noqa: E402

d = sys.argv[1]
for ns in ('leo1', 'leo2'):
    stem = os.path.join(d, f'{ns}_shared_map')
    if not os.path.exists(stem + '.pgm'):
        print(f'{stem}.pgm missing')
        continue
    img = read_pgm(stem + '.pgm')
    plt.figure(figsize=(10, 7))
    plt.imshow(img, cmap='gray', origin='lower')
    plt.title(f'{ns}/shared_map — merged locally on {ns}, own frame')
    plt.tight_layout()
    plt.savefig(stem + '.png', dpi=90)
    plt.close()
    print('wrote', stem + '.png')
