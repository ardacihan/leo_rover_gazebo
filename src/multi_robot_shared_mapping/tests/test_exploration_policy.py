"""Tests for collaborative exploration confidence policy."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.exploration_policy import (
    build_policy_debug,
    classify_confidence_level,
    exploration_allowed,
    min_acceptance_confidence,
)


def test_map_only_medium_without_tags():
    level = classify_confidence_level(
        0.55, 0, map_overlap_score=0.45, ambiguity_score=0.2, is_ambiguous=False
    )
    assert level == "medium"
    assert exploration_allowed(level)


def test_one_tag_with_map_agreement_medium():
    level = classify_confidence_level(
        0.45,
        1,
        map_overlap_score=0.35,
        ambiguity_score=0.1,
        tag_map_agreement=True,
    )
    assert level == "medium"


def test_one_tag_alone_not_high():
    level = classify_confidence_level(
        0.8, 1, map_overlap_score=0.1, ambiguity_score=0.0, tag_map_agreement=False
    )
    assert level != "high"


def test_ambiguous_is_low_and_blocks_exploration():
    level = classify_confidence_level(
        0.7, 0, map_overlap_score=0.5, ambiguity_score=0.9, is_ambiguous=True
    )
    assert level == "low"
    assert not exploration_allowed(level, is_ambiguous=True)


def test_map_only_acceptance_lower_when_strong_overlap():
    threshold = min_acceptance_confidence(
        0, "map", 0.50, is_ambiguous=False, map_mode_min=0.6
    )
    assert threshold < 0.6


def test_one_tag_acceptance_with_agreement():
    threshold = min_acceptance_confidence(
        1, "hybrid", 0.30, tag_map_agreement=True, base_min=0.5
    )
    assert threshold < 0.5


def test_debug_json_fields():
    policy = build_policy_debug(
        mode="hybrid",
        final_confidence=0.5,
        common_landmark_count=0,
        map_overlap_score=0.42,
        ambiguity_score=0.2,
        is_ambiguous=False,
        tag_map_agreement=False,
        leo1_quality=0.9,
        leo2_quality=0.9,
        recovery=None,
    )
    assert "exploration_allowed" in policy
    assert "confidence_level" in policy
    assert "recommended_action" in policy
    assert policy["recommended_action"] == "explore_separate_frontiers"
