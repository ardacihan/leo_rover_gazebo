#!/usr/bin/env python3
"""Unit tests for the persistent AprilTag landmark map (no ROS required)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.tag_landmark_map import (  # noqa: E402
    LandmarkMap,
    compute_tag_alignment_confidence,
    geometric_spread,
    merge_landmark_dicts,
)


def make_map_with_tag(tag_id=0, x=1.0, y=2.0, observations=1):
    landmark_map = LandmarkMap()
    for i in range(observations):
        landmark_map.update(tag_id, x, y, 0.0, stamp=float(i))
    return landmark_map


def test_add_and_persist():
    landmark_map = make_map_with_tag()
    assert 0 in landmark_map.landmarks
    # Landmarks must survive arbitrary time gaps (no timeout-based deletion).
    landmark_map.update(1, 5.0, 5.0, 0.0, stamp=10_000.0)
    assert 0 in landmark_map.landmarks and 1 in landmark_map.landmarks


def test_reobservation_refines_position():
    landmark_map = make_map_with_tag(x=1.0, y=1.0)
    landmark_map.update(0, 1.2, 1.0, 0.0, stamp=1.0)
    lm = landmark_map.landmarks[0]
    assert 1.0 < lm.x < 1.2
    assert lm.observation_count == 2
    assert lm.last_seen_time == 1.0


def test_confidence_grows_with_observations():
    few = make_map_with_tag(observations=2).landmarks[0].confidence
    many = make_map_with_tag(observations=10).landmarks[0].confidence
    assert many > few


def test_outlier_gated_for_mature_landmark():
    landmark_map = make_map_with_tag(x=1.0, y=1.0, observations=6)
    before = landmark_map.landmarks[0]
    conf_before = before.confidence
    outcome = landmark_map.update(0, 9.0, 9.0, 0.0, stamp=7.0)
    after = landmark_map.landmarks[0]
    assert outcome == "rejected"
    assert after.x == pytest.approx(1.0)
    assert after.confidence < conf_before


def test_young_landmark_accepts_jump():
    landmark_map = make_map_with_tag(x=1.0, y=1.0, observations=1)
    outcome = landmark_map.update(0, 3.0, 1.0, 0.0, stamp=1.0)
    assert outcome == "updated"
    assert landmark_map.landmarks[0].x == pytest.approx(2.0)


def test_geometric_spread():
    assert geometric_spread([(0.0, 0.0)]) == 0.0
    spread = geometric_spread([(0.0, 0.0), (2.0, 0.0)])
    assert spread == pytest.approx(1.0)


def test_alignment_confidence_range_and_ordering():
    map1 = LandmarkMap()
    map2 = LandmarkMap()
    for tid, (x, y) in enumerate([(0.0, 0.0), (4.0, 0.0), (0.0, 4.0)]):
        for i in range(8):
            map1.update(tid, x, y, 0.0, float(i))
            map2.update(tid, x - 2.0, y + 1.0, 0.0, float(i))

    good = compute_tag_alignment_confidence(
        [0, 1, 2], map1, map2, 0.02, 0.05, (2.0, -1.0, 0.0), (2.0, -1.0, 0.0)
    )
    bad = compute_tag_alignment_confidence(
        [0, 1, 2], map1, map2, 0.5, 1.0, (0.0, 0.0, 0.0), (2.0, -1.0, 0.0)
    )
    assert 0.0 <= bad < good <= 1.0
    assert compute_tag_alignment_confidence(
        [], map1, map2, 0.0, 0.0, None, (0.0, 0.0, 0.0)
    ) == 0.0


def test_merge_landmark_dicts_transforms_and_combines():
    map1 = make_map_with_tag(tag_id=0, x=1.0, y=1.0, observations=5)
    map2 = LandmarkMap()
    for i in range(5):
        map2.update(0, 0.0, 0.0, 0.0, float(i))  # same tag at leo2 origin
        map2.update(7, 2.0, 0.0, 0.0, float(i))  # leo2-only tag

    merged = merge_landmark_dicts(
        map1.to_dict_list(), map2.to_dict_list(), (1.0, 1.0, 0.0)
    )
    by_id = {lm["tag_id"]: lm for lm in merged}

    # Common tag: both robots agree on (1, 1) after the transform.
    assert by_id[0]["x"] == pytest.approx(1.0, abs=1e-3)
    assert by_id[0]["y"] == pytest.approx(1.0, abs=1e-3)
    assert by_id[0]["observation_count"] == 10
    # leo2-only tag transformed into leo1 frame.
    assert by_id[7]["x"] == pytest.approx(3.0, abs=1e-3)
    assert by_id[7]["y"] == pytest.approx(1.0, abs=1e-3)


def test_merge_with_rotation():
    map1 = LandmarkMap()
    map2 = LandmarkMap()
    map2.update(3, 1.0, 0.0, 0.0, 0.0)
    merged = merge_landmark_dicts(
        map1.to_dict_list(), map2.to_dict_list(), (0.0, 0.0, math.pi / 2.0)
    )
    assert merged[0]["x"] == pytest.approx(0.0, abs=1e-3)
    assert merged[0]["y"] == pytest.approx(1.0, abs=1e-3)
