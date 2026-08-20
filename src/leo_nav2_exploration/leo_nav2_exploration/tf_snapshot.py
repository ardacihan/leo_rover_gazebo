"""Read a live transform without publishing or changing TF."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from .calibration_math import quaternion_to_euler


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print one live TF transform and RPY angles.")
    parser.add_argument("parent", help="Parent/target frame")
    parser.add_argument("child", help="Child/source frame")
    parser.add_argument("--timeout", type=float, default=8.0)
    return parser


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    rclpy.init(args=raw)
    node = Node("tf_snapshot")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + cli.timeout
    transform = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(
                    cli.parent,
                    cli.child,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                break
            except TransformException:
                continue
        if transform is None:
            print(f"ERROR: no transform {cli.parent} <- {cli.child} within {cli.timeout:.1f}s", file=sys.stderr)
            return 2
        t = transform.transform.translation
        q = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        print(f"parent: {cli.parent}")
        print(f"child: {cli.child}")
        print(f"translation_m: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
        print(f"quaternion_xyzw: [{q.x:.8f}, {q.y:.8f}, {q.z:.8f}, {q.w:.8f}]")
        print(
            "rpy_deg: "
            f"[{math.degrees(roll):.4f}, {math.degrees(pitch):.4f}, {math.degrees(yaw):.4f}]"
        )
        return 0
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
