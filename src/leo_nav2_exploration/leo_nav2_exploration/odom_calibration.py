"""Interactive straight-line or rotation odometry scale calibration."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .calibration_math import (
    linear_odometry_scale,
    normalize_angle,
    quaternion_to_euler,
    rotational_odometry_scale,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Observe odometry while you manually drive a tape-measured distance or rotation."
    )
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--mode", choices=("linear", "angular"), required=True)
    parser.add_argument(
        "--actual",
        type=float,
        required=True,
        help="Actual tape-measured metres for linear mode, or degrees for angular mode",
    )
    parser.add_argument("--data-timeout", type=float, default=10.0)
    return parser


@dataclass(frozen=True)
class OdomSnapshot:
    x: float
    y: float
    cumulative_yaw: float


class OdomObserver(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("odom_calibration")
        self.latest: Optional[OdomSnapshot] = None
        self._previous_yaw: Optional[float] = None
        self._cumulative_yaw = 0.0
        self.create_subscription(Odometry, topic, self._callback, 50)

    def _callback(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        try:
            _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        except ValueError:
            return
        if self._previous_yaw is not None:
            self._cumulative_yaw += normalize_angle(yaw - self._previous_yaw)
        self._previous_yaw = yaw
        self.latest = OdomSnapshot(
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            cumulative_yaw=self._cumulative_yaw,
        )


def _copy(snapshot: Optional[OdomSnapshot]) -> Optional[OdomSnapshot]:
    if snapshot is None:
        return None
    return OdomSnapshot(snapshot.x, snapshot.y, snapshot.cumulative_yaw)


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    if cli.actual <= 0.0:
        print("ERROR: --actual must be positive", file=sys.stderr)
        return 2

    rclpy.init(args=raw)
    node = OdomObserver(cli.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        deadline = time.monotonic() + cli.data_timeout
        while node.latest is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if node.latest is None:
            print(f"ERROR: no odometry received on {cli.topic}", file=sys.stderr)
            return 3

        print("Keep autonomous navigation stopped during this calibration.")
        input("Place the rover at the marked start pose, then press Enter to capture START... ")
        start = _copy(node.latest)
        if cli.mode == "linear":
            input(
                f"Drive as straight as possible for exactly {cli.actual:.4f} m, stop, then press Enter... "
            )
        else:
            input(
                f"Rotate in place through exactly {cli.actual:.3f} degrees, stop, then press Enter... "
            )
        end = _copy(node.latest)
        if start is None or end is None:
            print("ERROR: odometry disappeared during calibration", file=sys.stderr)
            return 4

        if cli.mode == "linear":
            reported = math.hypot(end.x - start.x, end.y - start.y)
            scale = linear_odometry_scale(cli.actual, reported)
            print(f"reported_linear_distance_m: {reported:.6f}")
            print(f"actual_linear_distance_m: {cli.actual:.6f}")
            print(f"linear_scale_actual_over_reported: {scale:.8f}")
        else:
            reported = abs(end.cumulative_yaw - start.cumulative_yaw)
            actual = math.radians(cli.actual)
            scale = rotational_odometry_scale(actual, reported)
            print(f"reported_rotation_deg: {math.degrees(reported):.6f}")
            print(f"actual_rotation_deg: {cli.actual:.6f}")
            print(f"angular_scale_actual_over_reported: {scale:.8f}")
        print(
            "Apply a scale only at the wheel-odometry source or estimator. Do not add a second odom->base TF publisher."
        )
        return 0
    except (EOFError, KeyboardInterrupt):
        print("Calibration cancelled", file=sys.stderr)
        return 130
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
