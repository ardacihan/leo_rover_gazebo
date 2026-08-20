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


@dataclass(frozen=True)
class GuardState:
    now: float
    command_stamp: Optional[float]
    scan_stamp: Optional[float]
    odom_stamp: Optional[float]
    battery_stamp: Optional[float] = None
    battery_voltage: Optional[float] = None
    scan_valid_points: Optional[int] = None


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
    return GuardDecision(
        linear_x=linear_x,
        angular_z=angular_z,
        permitted=True,
        reason="permitted_clamped" if clamped else "permitted",
    )
