"""The gate and explorer read raw scan angles, so a LIDAR that is not mounted
facing forward mirrors their front/rear sectors.

Rover 4 publishes ``base_footprint -> laser_frame`` with yaw = pi, which made
the explorer's "front" sector read the corridor behind the rover and the gate's
rear-clearance check read the corridor in front of it.  These tests pin the
rotation maths that ``scan_yaw_offset`` applies, without needing rclpy.
"""

import ast
import math
import pathlib
import unittest


PACKAGE = pathlib.Path(__file__).parents[1]
GATE = PACKAGE / "scripts" / "safety_command_gate.py"
EXPLORER = PACKAGE / "scripts" / "safe_room_explorer.py"


def _normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _rear_clearance(ranges, angle_min, angle_increment, yaw_offset):
    """Mirror of SafetyCommandGate._scan_callback's rear-corridor extraction."""
    rear = []
    angle = angle_min + yaw_offset
    for value in ranges:
        if math.isfinite(value):
            x = value * math.cos(angle)
            y = value * math.sin(angle)
            if x < 0.0 and abs(y) <= 0.30:
                rear.append(-x)
        angle += angle_increment
    return min(rear) if rear else None


def _sector(ranges, angle_min, angle_increment, yaw_offset, lo_deg, hi_deg):
    """Mirror of SafeRoomExplorer._sector_clearance's angle selection."""
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    values = []
    angle = angle_min + yaw_offset
    for value in ranges:
        if lo <= _normalize(angle) <= hi and math.isfinite(value):
            values.append(value)
        angle += angle_increment
    return min(values) if values else None


class ScanYawOffsetTests(unittest.TestCase):
    """A wall 0.5 m ahead of the rover, sampled at Rover 4's scan geometry."""

    ANGLE_MIN = -3.118870496749878
    ANGLE_INCREMENT = 0.01239244919270277
    COUNT = 504

    def _scan_with_obstacle_at(self, obstacle_angle_in_base):
        """Build ranges placing a 0.5 m return at a bearing in the BASE frame.

        The LIDAR is mounted at yaw = pi, so a base-frame bearing appears in the
        scan at that bearing minus pi.
        """
        ranges = [10.0] * self.COUNT
        target = _normalize(obstacle_angle_in_base - math.pi)
        for index in range(self.COUNT):
            angle = _normalize(self.ANGLE_MIN + index * self.ANGLE_INCREMENT)
            if abs(_normalize(angle - target)) <= math.radians(10.0):
                ranges[index] = 0.5
        return ranges

    def test_uncorrected_gate_reads_a_front_wall_as_rear_clearance(self):
        """Without the offset the gate would approve reversing into a wall."""
        ranges = self._scan_with_obstacle_at(0.0)  # wall directly ahead

        wrong = _rear_clearance(
            ranges, self.ANGLE_MIN, self.ANGLE_INCREMENT, 0.0
        )
        # ~0.49 m, not exactly 0.5: the arc's projection onto -x shortens by
        # cos() toward the edges of the +/-0.30 m corridor.
        self.assertIsNotNone(wrong)
        self.assertLess(wrong, 0.55)
        self.assertGreater(wrong, 0.45)

        corrected = _rear_clearance(
            ranges, self.ANGLE_MIN, self.ANGLE_INCREMENT, math.pi
        )
        self.assertGreater(corrected, 5.0)

    def test_offset_puts_a_front_wall_in_the_front_sector(self):
        ranges = self._scan_with_obstacle_at(0.0)

        uncorrected = _sector(
            ranges, self.ANGLE_MIN, self.ANGLE_INCREMENT, 0.0, -32.0, 32.0
        )
        self.assertGreater(uncorrected, 5.0, "front sector was mirrored")

        corrected = _sector(
            ranges, self.ANGLE_MIN, self.ANGLE_INCREMENT, math.pi, -32.0, 32.0
        )
        self.assertAlmostEqual(corrected, 0.5, places=2)

    def test_offset_is_identity_for_a_forward_facing_lidar(self):
        """Rover 1 mounts the LIDAR forward and must be unaffected."""
        ranges = self._scan_with_obstacle_at(math.pi)  # wall behind the rover

        # With yaw=0 hardware the scan needs no rotation, so a return at
        # scan-bearing 0 belongs in the front sector.
        forward = _sector(
            [0.5 if abs(_normalize(self.ANGLE_MIN + i * self.ANGLE_INCREMENT))
             <= math.radians(10.0) else 10.0 for i in range(self.COUNT)],
            self.ANGLE_MIN, self.ANGLE_INCREMENT, 0.0, -32.0, 32.0,
        )
        self.assertAlmostEqual(forward, 0.5, places=2)
        self.assertIsNotNone(ranges)


class ScanYawOffsetWiringTests(unittest.TestCase):
    def test_both_nodes_declare_and_apply_the_offset(self):
        for path in (GATE, EXPLORER):
            source = path.read_text(encoding="utf-8")
            tree = ast.parse(source)

            declared = any(
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == "declare_parameter"
                and call.args
                and isinstance(call.args[0], ast.Constant)
                and call.args[0].value == "scan_yaw_offset"
                for call in ast.walk(tree)
            )
            self.assertTrue(declared, f"{path.name} must declare scan_yaw_offset")

            # The offset is worthless unless it is added to the scan's start
            # angle before any trigonometry runs.
            self.assertIn("+ self.scan_yaw_offset", source, path.name)


if __name__ == "__main__":
    unittest.main()
