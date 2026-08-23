import pytest

from leo_nav2_exploration.velocity_guard_logic import (
    GuardConfig,
    GuardState,
    evaluate_guard,
)


def config(**overrides):
    values = dict(
        max_linear_speed=0.10,
        max_angular_speed=0.30,
        command_timeout=0.50,
        scan_timeout=0.50,
        odom_timeout=0.75,
        require_battery=False,
        battery_timeout=2.0,
        min_battery_voltage=11.0,
        minimum_valid_scan_points=30,
    )
    values.update(overrides)
    return GuardConfig(**values)


def state(**overrides):
    values = dict(
        now=10.0,
        command_stamp=9.8,
        scan_stamp=9.8,
        odom_stamp=9.7,
        battery_stamp=None,
        battery_voltage=None,
        scan_valid_points=100,
    )
    values.update(overrides)
    return GuardState(**values)


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("command_stamp", None, "command_missing"),
        ("command_stamp", 9.0, "command_stale"),
        ("scan_stamp", None, "scan_missing"),
        ("scan_stamp", 9.0, "scan_stale"),
        ("odom_stamp", None, "odom_missing"),
        ("odom_stamp", 9.0, "odom_stale"),
    ],
)
def test_missing_or_stale_required_inputs_fail_closed(field, value, reason):
    decision = evaluate_guard(0.05, 0.1, config(), state(**{field: value}))
    assert not decision.permitted
    assert decision.reason == reason
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0



def test_fresh_scan_with_too_few_valid_points_fails_closed():
    decision = evaluate_guard(
        0.05,
        0.0,
        config(minimum_valid_scan_points=30),
        state(scan_valid_points=4),
    )
    assert not decision.permitted
    assert decision.reason == "scan_insufficient"
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0


def test_battery_is_not_required_in_baseline_profiles():
    decision = evaluate_guard(0.05, 0.1, config(require_battery=False), state())
    assert decision.permitted


@pytest.mark.parametrize(
    ("updates", "reason"),
    [
        ({"battery_stamp": None, "battery_voltage": None}, "battery_missing"),
        ({"battery_stamp": 7.0, "battery_voltage": 12.0}, "battery_stale"),
        ({"battery_stamp": 9.9, "battery_voltage": 10.8}, "battery_low"),
    ],
)
def test_required_battery_faults_fail_closed(updates, reason):
    decision = evaluate_guard(
        0.05,
        0.1,
        config(require_battery=True),
        state(**updates),
    )
    assert not decision.permitted
    assert decision.reason == reason


def test_valid_command_is_clamped_symmetrically():
    decision = evaluate_guard(0.40, -1.2, config(), state())
    assert decision.permitted
    assert decision.reason == "permitted_clamped"
    assert decision.linear_x == pytest.approx(0.10)
    assert decision.angular_z == pytest.approx(-0.30)


def test_valid_command_passes_unchanged_inside_limits():
    decision = evaluate_guard(-0.04, 0.22, config(), state())
    assert decision.permitted
    assert decision.reason == "permitted"
    assert decision.linear_x == pytest.approx(-0.04)
    assert decision.angular_z == pytest.approx(0.22)


def test_future_timestamp_is_rejected_as_clock_error():
    decision = evaluate_guard(0.05, 0.0, config(), state(scan_stamp=10.2))
    assert not decision.permitted
    assert decision.reason == "scan_clock_error"
