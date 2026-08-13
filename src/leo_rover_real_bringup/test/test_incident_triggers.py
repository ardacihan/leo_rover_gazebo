#!/usr/bin/env python3

"""Decision tests for debug-capture triggers."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from incident_triggers import (
    gate_block,
    monitor_veto,
    near_miss,
    sensor_conflict,
    should_capture,
)


class MonitorVetoTest(unittest.TestCase):
    def test_fires_when_the_monitor_zeroes_a_real_command(self):
        self.assertTrue(monitor_veto(0.08, 0.0))

    def test_fires_on_a_material_slowdown(self):
        self.assertTrue(monitor_veto(0.08, 0.02))

    def test_quiet_when_the_command_passes_through(self):
        self.assertFalse(monitor_veto(0.08, 0.08))

    def test_quiet_when_nothing_was_commanded(self):
        """An idle rover is not being vetoed."""
        self.assertFalse(monitor_veto(0.0, 0.0))


class GateBlockTest(unittest.TestCase):
    def test_fires_when_a_request_produces_nothing(self):
        self.assertTrue(gate_block(0.08, 0.0, 0.0, 0.0))

    def test_fires_for_a_blocked_turn(self):
        self.assertTrue(gate_block(0.0, 0.25, 0.0, 0.0))

    def test_quiet_when_the_gate_passes_it(self):
        self.assertFalse(gate_block(0.08, 0.0, 0.08, 0.0))

    def test_quiet_when_idle(self):
        self.assertFalse(gate_block(0.0, 0.0, 0.0, 0.0))


class NearMissTest(unittest.TestCase):
    def test_fires_inside_the_threshold(self):
        self.assertTrue(near_miss(0.35, 0.50))

    def test_quiet_outside(self):
        self.assertFalse(near_miss(1.20, 0.50))

    def test_zero_means_no_data_not_a_collision(self):
        self.assertFalse(near_miss(0.0, 0.50))


class SensorConflictTest(unittest.TestCase):
    def test_fires_when_camera_sees_an_obstacle_lidar_misses(self):
        self.assertTrue(sensor_conflict(2.50, 1.00))

    def test_quiet_on_close_agreement(self):
        self.assertFalse(sensor_conflict(1.00, 1.05))

    def test_quiet_beyond_camera_range(self):
        """Past usable depth the two legitimately diverge."""
        self.assertFalse(sensor_conflict(6.00, 4.00, valid_max=3.0))

    def test_quiet_when_a_sensor_has_no_data(self):
        self.assertFalse(sensor_conflict(0.0, 1.0))


class RateLimitTest(unittest.TestCase):
    def test_first_capture_of_a_kind_always_allowed(self):
        self.assertTrue(should_capture(100.0, {}, "monitor_veto", 1.5))

    def test_suppressed_inside_the_interval(self):
        self.assertFalse(
            should_capture(100.5, {"monitor_veto": 100.0}, "monitor_veto", 1.5)
        )

    def test_allowed_again_after_the_interval(self):
        self.assertTrue(
            should_capture(102.0, {"monitor_veto": 100.0}, "monitor_veto", 1.5)
        )

    def test_kinds_are_limited_independently(self):
        """A sustained veto must not suppress a distinct sensor conflict."""
        last = {"monitor_veto": 100.0}
        self.assertTrue(
            should_capture(100.2, last, "sensor_conflict", 1.5)
        )


if __name__ == "__main__":
    unittest.main()
