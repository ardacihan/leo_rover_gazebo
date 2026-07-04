#!/usr/bin/env python3
"""
Accepted-vs-candidate transform state machine (no ROS imports).

The shared map must never jump to every new estimate. Candidates are tested
against the currently accepted transform; only clear improvements are promoted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

Transform = Tuple[float, float, float]  # dx, dy, yaw


@dataclass
class AlignmentState:
    """Tracks accepted transform and the latest candidate evaluation."""

    accepted: Optional[Transform] = None
    accepted_confidence: float = 0.0
    candidate: Optional[Transform] = None
    candidate_confidence: float = 0.0
    candidate_reason: str = ""
    last_rejection_reason: str = ""

    min_alignment_confidence: float = 0.5
    min_confidence_improvement: float = 0.05
    max_transform_jump: float = 2.0
    max_yaw_jump: float = math.radians(25.0)
    require_consistency_for_update: bool = True

    def evaluate_candidate(
        self,
        candidate: Transform,
        confidence: float,
        *,
        extra_reject_reason: str = "",
    ) -> Tuple[bool, str]:
        """
        Decide whether to promote candidate to accepted.

        Returns (accepted_now, reason). On rejection the previous accepted
        transform is kept unchanged.
        """
        self.candidate = candidate
        self.candidate_confidence = confidence

        if extra_reject_reason:
            self.last_rejection_reason = extra_reject_reason
            self.candidate_reason = extra_reject_reason
            return False, extra_reject_reason

        if confidence < self.min_alignment_confidence:
            reason = (
                f"confidence {confidence:.2f} < min_alignment_confidence "
                f"{self.min_alignment_confidence:.2f}"
            )
            self.last_rejection_reason = reason
            self.candidate_reason = reason
            return False, reason

        if self.accepted is not None and self.require_consistency_for_update:
            jump = math.hypot(
                candidate[0] - self.accepted[0], candidate[1] - self.accepted[1]
            )
            yaw_jump = abs(_normalize_angle(candidate[2] - self.accepted[2]))
            if jump > self.max_transform_jump:
                reason = (
                    f"transform jump {jump:.2f} m > max_transform_jump "
                    f"{self.max_transform_jump:.2f}"
                )
                self.last_rejection_reason = reason
                self.candidate_reason = reason
                return False, reason
            if yaw_jump > self.max_yaw_jump:
                reason = (
                    f"yaw jump {math.degrees(yaw_jump):.1f} deg > "
                    f"max_yaw_jump_deg {math.degrees(self.max_yaw_jump):.1f}"
                )
                self.last_rejection_reason = reason
                self.candidate_reason = reason
                return False, reason

            if confidence < self.accepted_confidence + self.min_confidence_improvement:
                reason = (
                    f"confidence {confidence:.2f} does not improve accepted "
                    f"{self.accepted_confidence:.2f} by "
                    f"min_confidence_improvement {self.min_confidence_improvement:.2f}"
                )
                self.last_rejection_reason = reason
                self.candidate_reason = reason
                return False, reason

        self.accepted = candidate
        self.accepted_confidence = confidence
        self.candidate_reason = "accepted"
        self.last_rejection_reason = ""
        return True, "accepted"

    def debug_dict(self, mode: str, **extra) -> dict:
        """Build the /alignment_debug_json payload."""
        out = {
            "mode": mode,
            "candidate_dx": self.candidate[0] if self.candidate else None,
            "candidate_dy": self.candidate[1] if self.candidate else None,
            "candidate_yaw": self.candidate[2] if self.candidate else None,
            "accepted_dx": self.accepted[0] if self.accepted else None,
            "accepted_dy": self.accepted[1] if self.accepted else None,
            "accepted_yaw": self.accepted[2] if self.accepted else None,
            "accepted": self.candidate_reason == "accepted",
            "reason": self.candidate_reason or self.last_rejection_reason,
        }
        out.update(extra)
        return out


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
