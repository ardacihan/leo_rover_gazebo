"""Estimate LiDAR zero-angle and range against a flat board."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

from .calibration_math import fit_board_from_scan, normalize_angle


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fit a flat board in LaserScan data. Centre a wide board perpendicular to the rover forward axis."
    )
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--expected-angle-deg", type=float, default=0.0)
    parser.add_argument("--half-width-deg", type=float, default=30.0)
    parser.add_argument("--minimum-range", type=float, default=0.20)
    parser.add_argument("--maximum-range", type=float, default=3.0)
    parser.add_argument("--range-tolerance", type=float, default=0.30)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--known-distance",
        type=float,
        help="Tape-measured perpendicular distance from the LiDAR centre to the board, metres",
    )
    return parser


class LidarBoardCollector(Node):
    def __init__(self, cli) -> None:
        super().__init__("lidar_board_calibration")
        self.cli = cli
        self.fits = []
        self.rejected = 0
        self.create_subscription(LaserScan, cli.topic, self._scan, qos_profile_sensor_data)

    def _scan(self, message: LaserScan) -> None:
        if len(self.fits) >= self.cli.samples:
            return
        minimum = max(self.cli.minimum_range, float(message.range_min))
        maximum = min(self.cli.maximum_range, float(message.range_max))
        try:
            fit = fit_board_from_scan(
                ranges=message.ranges,
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                expected_angle=math.radians(self.cli.expected_angle_deg),
                half_width=math.radians(self.cli.half_width_deg),
                minimum_range=minimum,
                maximum_range=maximum,
                range_tolerance=self.cli.range_tolerance,
            )
        except ValueError:
            self.rejected += 1
            return
        self.fits.append(fit)


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    if cli.samples < 5 or cli.timeout <= 0.0:
        print("ERROR: samples must be at least 5 and timeout must be positive", file=sys.stderr)
        return 2
    rclpy.init(args=raw)
    node = LidarBoardCollector(cli)
    deadline = time.monotonic() + cli.timeout
    try:
        while rclpy.ok() and len(node.fits) < cli.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.fits) < 5:
            print(
                f"ERROR: only {len(node.fits)} usable board fits; rejected {node.rejected}. "
                "Use a larger board, remove nearby clutter, or narrow the sector.",
                file=sys.stderr,
            )
            return 3

        angles = [fit.normal_angle for fit in node.fits]
        reference = angles[0]
        unwrapped = [reference + normalize_angle(angle - reference) for angle in angles]
        angle = statistics.median(unwrapped)
        distance = statistics.median(fit.distance for fit in node.fits)
        rms = statistics.median(fit.rms_error for fit in node.fits)
        yaw_mount = -angle

        print(f"usable_scans: {len(node.fits)}")
        print(f"rejected_scans: {node.rejected}")
        print(f"board_distance_from_lidar_m: {distance:.5f}")
        print(f"board_normal_angle_in_scan_deg: {math.degrees(angle):.4f}")
        print(f"median_line_rms_m: {rms:.5f}")
        print(f"recommended_base_to_laser_yaw_deg: {math.degrees(yaw_mount):.4f}")
        print(
            "Interpretation: when the board is physically perpendicular to rover forward, "
            "base->laser yaw is approximately the negative of the measured scan angle."
        )
        if cli.known_distance is not None:
            if cli.known_distance <= 0.0:
                print("ERROR: known distance must be positive", file=sys.stderr)
                return 4
            print(f"range_scale_actual_over_reported: {cli.known_distance / distance:.8f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
