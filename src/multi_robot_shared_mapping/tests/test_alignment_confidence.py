"""Tests for alignment confidence scoring and ambiguity detection."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.alignment_confidence import (
    ConfidenceInputs,
    compute_final_confidence,
    detect_ambiguity,
    select_top_candidates,
    transforms_disagree,
)


def test_compute_final_confidence_high_when_all_good():
    inputs = ConfidenceInputs(
        occupancy_overlap_score=0.9,
        free_space_conflict_score=0.9,
        transform_stability_score=0.9,
        unambiguity_score=0.9,
        local_map_quality_score=0.9,
        tag_alignment_confidence=0.8,
    )
    assert compute_final_confidence(inputs) > 0.85


def test_compute_final_confidence_low_on_conflict():
    inputs = ConfidenceInputs(
        occupancy_overlap_score=0.2,
        free_space_conflict_score=0.1,
        transform_stability_score=0.5,
        unambiguity_score=0.5,
        local_map_quality_score=0.5,
    )
    assert compute_final_confidence(inputs) < 0.35


def test_detect_ambiguity_when_scores_close():
    candidates = [
        (0.0, 0.0, 0.0, 100.0),
        (5.0, 0.0, 0.0, 90.0),
    ]
    ratio, ambiguous = detect_ambiguity(candidates, ambiguity_ratio_threshold=0.85)
    assert ambiguous
    assert ratio == 0.9


def test_detect_not_ambiguous_when_runner_up_far():
    candidates = [
        (0.0, 0.0, 0.0, 100.0),
        (0.1, 0.0, 0.0, 95.0),
    ]
    _, ambiguous = detect_ambiguity(candidates)
    assert not ambiguous


def test_select_top_candidates_nms():
    candidates = [
        (0.0, 0.0, 0.0, 10.0),
        (0.05, 0.0, 0.0, 9.0),
        (3.0, 0.0, 0.0, 8.0),
    ]
    top = select_top_candidates(candidates, k=2, min_separation_m=0.5)
    assert len(top) == 2
    assert top[0][3] == 10.0
    assert top[1][0] == 3.0


def test_transforms_disagree():
    a = (0.0, 0.0, 0.0)
    b = (2.0, 0.0, math.radians(20.0))
    assert transforms_disagree(a, b, max_translation_m=1.0, max_yaw=math.radians(15.0))
