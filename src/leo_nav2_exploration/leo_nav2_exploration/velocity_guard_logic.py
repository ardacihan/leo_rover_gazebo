"""Pure fail-closed command guard logic, independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class GuardConfig:
    max_linear_speed: float
    max_angular_speed: float
    command_timeout: float
    scan_timeout: float
    odom_timeout: float
    require_battery: bool = False
    battery_timeout: float = 2.0
    min_battery_voltage: float = 0.0
    minimum_valid_scan_points: int = 20
    future_tolerance: float = 0.05
    # 0 disables the cone stop. Real rover yaml sets ~0.42 m so a plain
    # wall in front zeros linear speed while still allowing a turn.
    front_stop_distance: float = 0.0
    stop_half_angle: float = 0.70
    stop_min_points: int = 3
    blocked_turn_speed: float = 0.35

    def __post_init__(self) -> None:
        positive = (
            "max_linear_speed",
            "max_angular_speed",
            "command_timeout",
            "scan_timeout",
            "odom_timeout",
            "battery_timeout",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.min_battery_voltage):
            raise ValueError("min_battery_voltage must be finite")
        if int(self.minimum_valid_scan_points) < 1:
            raise ValueError("minimum_valid_scan_points must be at least 1")
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and non-negative")
        if not math.isfinite(self.front_stop_distance) or self.front_stop_distance < 0.0:
            raise ValueError("front_stop_distance must be finite and non-negative")
        if not math.isfinite(self.stop_half_angle) or self.stop_half_angle <= 0.0:
            raise ValueError("stop_half_angle must be finite and positive")
        if int(self.stop_min_points) < 1:
            raise ValueError("stop_min_points must be at least 1")
        if not math.isfinite(self.blocked_turn_speed) or self.blocked_turn_speed < 0.0:
            raise ValueError("blocked_turn_speed must be finite and non-negative")


@dataclass(frozen=True)
class GuardState:
    now: float
    command_stamp: Optional[float]
    scan_stamp: Optional[float]
    odom_stamp: Optional[float]
    battery_stamp: Optional[float] = None
    battery_voltage: Optional[float] = None
    scan_valid_points: Optional[int] = None
    front_min_range: Optional[float] = None
    front_hit_points: int = 0
    rear_min_range: Optional[float] = None
    rear_hit_points: int = 0
    left_min_range: Optional[float] = None
    right_min_range: Optional[float] = None


@dataclass(frozen=True)
class GuardDecision:
    linear_x: float
    angular_z: float
    permitted: bool
    reason: str


def _freshness_reason(
    *,
    name: str,
    now: float,
    stamp: Optional[float],
    timeout: float,
    future_tolerance: float,
) -> Optional[str]:
    if stamp is None:
        return f"{name}_missing"
    if not math.isfinite(stamp) or not math.isfinite(now):
        return f"{name}_clock_error"
    age = now - stamp
    if age < -future_tolerance:
        return f"{name}_clock_error"
    if age > timeout:
        return f"{name}_stale"
    return None


def _stop(reason: str) -> GuardDecision:
    return GuardDecision(0.0, 0.0, False, reason)


def _clamp(value: float, limit: float) -> float:
    return min(max(float(value), -limit), limit)


def min_range_in_cone(
    ranges,
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    center: float,
    half_angle: float,
) -> tuple[Optional[float], int]:
    """Nearest finite return and hit count inside a heading cone."""
    nearest: Optional[float] = None
    hits = 0
    for index, raw in enumerate(ranges):
        if not math.isfinite(raw) or raw < range_min or raw > range_max:
            continue
        angle = angle_min + index * angle_increment
        delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
        if abs(delta) > half_angle:
            continue
        hits += 1
        if nearest is None or raw < nearest:
            nearest = raw
    return nearest, hits


def _cone_blocked(distance: Optional[float], hits: int, config: GuardConfig) -> bool:
    if distance is None or hits < config.stop_min_points:
        return False
    return distance <= config.front_stop_distance


def _clearer_turn(left: Optional[float], right: Optional[float]) -> float:
    """Positive yaw is left. Turn toward the side with more free space."""
    left_d = 100.0 if left is None else left
    right_d = 100.0 if right is None else right
    if left_d >= right_d:
        return 1.0
    return -1.0


def evaluate_guard(
    requested_linear_x: float,
    requested_angular_z: float,
    config: GuardConfig,
    state: GuardState,
) -> GuardDecision:
    """Validate freshness and limits, returning a zero command on every fault."""

    if not math.isfinite(requested_linear_x) or not math.isfinite(requested_angular_z):
        return _stop("command_nonfinite")

    required = (
        ("command", state.command_stamp, config.command_timeout),
        ("scan", state.scan_stamp, config.scan_timeout),
        ("odom", state.odom_stamp, config.odom_timeout),
    )
    for name, stamp, timeout in required:
        reason = _freshness_reason(
            name=name,
            now=state.now,
            stamp=stamp,
            timeout=timeout,
            future_tolerance=config.future_tolerance,
        )
        if reason is not None:
            return _stop(reason)

    if state.scan_valid_points is None or state.scan_valid_points < config.minimum_valid_scan_points:
        return _stop("scan_insufficient")

    if config.require_battery:
        reason = _freshness_reason(
            name="battery",
            now=state.now,
            stamp=state.battery_stamp,
            timeout=config.battery_timeout,
            future_tolerance=config.future_tolerance,
        )
        if reason is not None or state.battery_voltage is None:
            return _stop(reason or "battery_missing")
        if not math.isfinite(state.battery_voltage):
            return _stop("battery_invalid")
        if state.battery_voltage < config.min_battery_voltage:
            return _stop("battery_low")

    linear_x = _clamp(requested_linear_x, config.max_linear_speed)
    angular_z = _clamp(requested_angular_z, config.max_angular_speed)
    clamped = (
        not math.isclose(linear_x, requested_linear_x, abs_tol=1e-12)
        or not math.isclose(angular_z, requested_angular_z, abs_tol=1e-12)
    )
    reason = "permitted_clamped" if clamped else "permitted"
    if config.front_stop_distance > 0.0:
        if linear_x > 0.01 and _cone_blocked(
            state.front_min_range, state.front_hit_points, config
        ):
            linear_x = 0.0
            if abs(angular_z) < 0.05:
                angular_z = config.blocked_turn_speed * _clearer_turn(
                    state.left_min_range, state.right_min_range
                )
            reason = "front_obstacle"
        elif linear_x < -0.01 and _cone_blocked(
            state.rear_min_range, state.rear_hit_points, config
        ):
            linear_x = 0.0
            reason = "rear_obstacle"
    return GuardDecision(
        linear_x=linear_x,
        angular_z=angular_z,
        permitted=True,
        reason=reason,
    )
