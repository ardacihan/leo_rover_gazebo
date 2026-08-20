"""Estimate depth-camera height, roll, and pitch from a floor plane."""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .calibration_math import camera_level_from_floor_normal, fit_plane_ransac


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the floor in a ROS camera optical-frame PointCloud2. "
            "Keep the rover stationary on a flat floor."
        )
    )
    parser.add_argument("--topic", default="/camera/camera/depth/color/points")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--max-points", type=int, default=50000)
    parser.add_argument("--min-forward", type=float, default=0.35, help="Minimum optical z, metres")
    parser.add_argument("--max-forward", type=float, default=3.0, help="Maximum optical z, metres")
    parser.add_argument("--max-abs-right", type=float, default=1.5, help="Maximum |optical x|, metres")
    parser.add_argument("--min-down", type=float, default=0.05, help="Minimum optical y; floor is below image centre")
    parser.add_argument("--ransac-threshold", type=float, default=0.02)
    parser.add_argument("--ransac-iterations", type=int, default=350)
    parser.add_argument(
        "--max-floor-tilt-deg",
        type=float,
        default=50.0,
        help="Reject dominant planes whose normal is farther from optical up",
    )
    return parser


class CameraFloorCollector(Node):
    """Collect a bounded set of lower-image PointCloud2 samples."""

    def __init__(self, cli: argparse.Namespace) -> None:
        super().__init__("camera_floor_calibration")
        self.cli = cli
        self.frames = 0
        self.frame_id = ""
        self.points: list[np.ndarray] = []
        self.create_subscription(PointCloud2, cli.topic, self._cloud, qos_profile_sensor_data)

    def _cloud(self, message: PointCloud2) -> None:
        if self.frames >= self.cli.frames:
            return
        selected: list[tuple[float, float, float]] = []
        try:
            iterator = point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            per_frame_limit = max(1000, self.cli.max_points // max(1, self.cli.frames))
            for index, point in enumerate(iterator):
                if index % 4:
                    continue
                x, y, z = (float(point[0]), float(point[1]), float(point[2]))
                if not (self.cli.min_forward <= z <= self.cli.max_forward):
                    continue
                if abs(x) > self.cli.max_abs_right or y < self.cli.min_down:
                    continue
                selected.append((x, y, z))
                if len(selected) >= per_frame_limit:
                    break
        except Exception as exc:  # malformed clouds should not crash a calibration session
            self.get_logger().warning(f"PointCloud2 conversion failed: {exc}")
            return
        if len(selected) >= 100:
            self.points.append(np.asarray(selected, dtype=float))
            self.frames += 1
            self.frame_id = message.header.frame_id
            self.get_logger().info(
                f"accepted frame {self.frames}/{self.cli.frames}: {len(selected)} points"
            )


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    if cli.frames < 3 or cli.max_points < 1000:
        print("ERROR: use at least 3 frames and 1000 points", file=sys.stderr)
        return 2
    if cli.ransac_iterations < 10 or not 0.0 < cli.max_floor_tilt_deg < 90.0:
        print("ERROR: use at least 10 RANSAC iterations and floor tilt in (0, 90) degrees", file=sys.stderr)
        return 2

    rclpy.init(args=raw)
    node = CameraFloorCollector(cli)
    deadline = time.monotonic() + cli.timeout
    try:
        while rclpy.ok() and node.frames < cli.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.frames < 3:
            print(
                f"ERROR: only {node.frames} usable frames. Verify the PointCloud2 topic and that the floor is visible.",
                file=sys.stderr,
            )
            return 3
        points = np.vstack(node.points)
        if points.shape[0] > cli.max_points:
            rng = np.random.default_rng(17)
            points = points[rng.choice(points.shape[0], size=cli.max_points, replace=False)]
        plane = fit_plane_ransac(
            points,
            distance_threshold=cli.ransac_threshold,
            iterations=cli.ransac_iterations,
            seed=19,
            expected_normal=np.array([0.0, -1.0, 0.0]),
            maximum_normal_angle=math.radians(cli.max_floor_tilt_deg),
        )
        level = camera_level_from_floor_normal(plane.normal)
        inlier_fraction = plane.inlier_count / points.shape[0]
        print(f"cloud_frame: {node.frame_id}")
        print(f"sampled_points: {points.shape[0]}")
        print(f"floor_inliers: {plane.inlier_count}")
        print(f"floor_inlier_fraction: {inlier_fraction:.4f}")
        print(f"floor_plane_rms_m: {plane.rms_error:.5f}")
        print(f"camera_optical_height_above_floor_m: {plane.distance:.5f}")
        print(f"camera_roll_deg: {math.degrees(level.roll_rad):.4f}")
        print(f"camera_pitch_down_deg: {math.degrees(level.pitch_down_rad):.4f}")
        print(
            "Use these values to verify the URDF/static TF. The script reports the optical-frame origin; "
            "do not add a second TF publisher if the existing transform is merely wrong."
        )
        if inlier_fraction < 0.35:
            print(
                "WARNING: weak floor plane; repeat with more visible carpet/floor and fewer walls.",
                file=sys.stderr,
            )
            return 4
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
