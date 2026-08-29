"""Regression coverage for the presentation media helpers."""

from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))

from render_multirobot_media import (  # noqa: E402
    final_locked_transform,
    markers_for,
)
from render_timelapse import colourise  # noqa: E402
from phase2_metrics import alignment_locked, truth_union_area  # noqa: E402


def test_colourised_flipped_map_is_opencv_drawable():
    grid = np.array([[0, 100], [-1, 0]], dtype=np.int8)
    image = colourise(grid)

    assert image.flags.c_contiguous
    # OpenCV 4.5 rejects the negative-stride view returned by np.flipud.
    cv2.circle(image, (0, 0), 1, (0, 0, 255), -1)


def test_depot_marker_truth_is_expressed_in_leo1_map_frame():
    markers = {marker_id: (x, y) for marker_id, x, y in
               markers_for('depot_world')}

    # leo1 starts at world (0, 4.5), yaw 0. Marker 1 is at world y=6.88.
    assert markers[1] == pytest.approx((0.0, 2.38), abs=1e-6)
    # Marker 2 is at world (3, -6.88).
    assert markers[2] == pytest.approx((3.0, -11.38), abs=1e-6)


def test_renderer_uses_last_accepted_transform(tmp_path):
    (tmp_path / 'alignment.csv').write_text(
        'map_x,map_y,map_yaw_deg,locked\n'
        '1.0,2.0,3.0,1\n'
        '9.0,9.0,90.0,0\n'
        '4.0,5.0,6.0,1\n',
        encoding='utf-8')

    transform = final_locked_transform(str(tmp_path))
    assert transform == pytest.approx((4.0, 5.0, 6.0))


def test_nonrendezvous_union_scores_both_local_maps():
    grid = np.zeros((2, 2), dtype=np.int8)
    info = (0.0, 0.0, 0.05)

    # Depot's authored transform places these two tiny maps far apart. The
    # scoring union must retain all eight known cells even without a runtime
    # alignment lock: 8 * 0.05^2 = 0.02 m2.
    area = truth_union_area(grid, info, grid, info, 'depot_world')
    assert area == pytest.approx(0.02)


def test_alignment_lock_detection(tmp_path):
    (tmp_path / 'alignment.csv').write_text(
        't,locked\n10,0\n20,1\n', encoding='utf-8')
    assert alignment_locked(str(tmp_path))
