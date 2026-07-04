"""Tests for accepted-vs-candidate alignment state machine."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.alignment_state import AlignmentState


def test_first_candidate_accepted_when_confident():
    state = AlignmentState(min_alignment_confidence=0.5)
    ok, reason = state.evaluate_candidate((1.0, 2.0, 0.1), 0.7)
    assert ok
    assert reason == "accepted"
    assert state.accepted == (1.0, 2.0, 0.1)
    assert state.accepted_confidence == 0.7


def test_low_confidence_rejected():
    state = AlignmentState(min_alignment_confidence=0.5)
    ok, reason = state.evaluate_candidate((1.0, 2.0, 0.0), 0.3)
    assert not ok
    assert "min_alignment_confidence" in reason
    assert state.accepted is None


def test_transform_jump_rejected():
    state = AlignmentState(
        min_alignment_confidence=0.5,
        max_transform_jump=1.0,
        require_consistency_for_update=True,
    )
    state.evaluate_candidate((0.0, 0.0, 0.0), 0.8)
    ok, reason = state.evaluate_candidate((3.0, 0.0, 0.0), 0.9)
    assert not ok
    assert "transform jump" in reason
    assert state.accepted == (0.0, 0.0, 0.0)


def test_confidence_must_improve():
    state = AlignmentState(
        min_alignment_confidence=0.5,
        min_confidence_improvement=0.1,
    )
    state.evaluate_candidate((0.0, 0.0, 0.0), 0.8)
    ok, reason = state.evaluate_candidate((0.1, 0.0, 0.0), 0.85)
    assert not ok
    assert "does not improve" in reason


def test_extra_reject_reason():
    state = AlignmentState()
    ok, reason = state.evaluate_candidate((0.0, 0.0, 0.0), 0.9, extra_reject_reason="ambiguous")
    assert not ok
    assert reason == "ambiguous"


def test_debug_dict_includes_transforms():
    state = AlignmentState()
    state.evaluate_candidate((1.0, 2.0, math.pi / 4), 0.75)
    d = state.debug_dict("hybrid", final_confidence=0.75)
    assert d["mode"] == "hybrid"
    assert d["accepted_dx"] == 1.0
    assert d["accepted"] is True
