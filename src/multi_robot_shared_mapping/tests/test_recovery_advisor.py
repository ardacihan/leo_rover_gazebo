"""Tests for low-confidence recovery recommendations."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.recovery_advisor import recommend_recovery


def test_no_recovery_when_exploration_allowed():
    assert recommend_recovery(
        confidence=0.55,
        min_confidence=0.5,
        common_ids=[0],
        leo1_landmarks={0: (0.0, 0.0)},
        leo2_landmarks={0: (0.0, 0.0)},
        exploration_allowed=True,
    ) is None


def test_no_recovery_when_confidence_ok():
    assert recommend_recovery(
        confidence=0.8,
        min_confidence=0.5,
        common_ids=[0, 1],
        leo1_landmarks={0: (1.0, 0.0), 1: (5.0, 0.0)},
        leo2_landmarks={0: (1.0, 0.0), 1: (5.0, 0.0)},
    ) is None


def test_zero_tags_suggests_separate_exploration_not_blocking():
    rec = recommend_recovery(
        confidence=0.2,
        min_confidence=0.5,
        common_ids=[],
        leo1_landmarks={},
        leo2_landmarks={},
        exploration_allowed=False,
    )
    assert rec is not None
    assert "no tag revisit required" in rec["reason"].lower() or rec[
        "recommended_action"
    ] in ("explore_separate_frontiers", "explore_overlap_frontier")


def test_one_common_tag_recovery_is_optional():
    rec = recommend_recovery(
        confidence=0.2,
        min_confidence=0.5,
        common_ids=[0],
        leo1_landmarks={0: (0.0, 0.0)},
        leo2_landmarks={0: (0.0, 0.0), 2: (3.0, 1.0)},
        leo2_pose=(0.0, 0.0),
        exploration_allowed=False,
    )
    assert rec is not None
    assert "optional" in rec["reason"].lower()


def test_ambiguous_recommends_distinctive_area():
    rec = recommend_recovery(
        confidence=0.2,
        min_confidence=0.5,
        common_ids=[],
        leo1_landmarks={},
        leo2_landmarks={},
        is_ambiguous=True,
        exploration_allowed=False,
    )
    assert rec["recommended_action"] == "go_to_distinctive_area"


def test_poor_local_map_recommends_relocalization():
    rec = recommend_recovery(
        confidence=0.2,
        min_confidence=0.5,
        common_ids=[0, 1],
        leo1_landmarks={0: (0.0, 0.0), 1: (1.0, 0.0)},
        leo2_landmarks={0: (0.0, 0.0), 1: (1.0, 0.0)},
        leo2_quality=0.1,
        reset_bad_local_map_recommended=True,
        exploration_allowed=False,
    )
    assert rec["recommended_robot"] == "leo2"
    assert "Restart/reinitialize local SLAM" in rec["reason"]
