"""Run a repeatable sequence of alternating Nav2 doorway goals."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .geometry import compose_planar_pose
from .navigate_goal import NavigateGoalClient


def _parser() -> argparse.ArgumentParser:
    default = Path(get_package_share_directory("leo_nav2_exploration")) / "config" / "sim" / "doorway_goals.yaml"
    parser = argparse.ArgumentParser(description="Execute all goals in the doorway regression YAML.")
    parser.add_argument("--scenario", type=Path, default=default)
    parser.add_argument("--output", type=Path, help="Optional JSON result file")
    parser.add_argument("--initial-wait", type=float, default=5.0)
    parser.add_argument("--continue-on-failure", action="store_true")
    return parser


def _spin_wait(node, duration: float) -> None:
    deadline = time.monotonic() + max(0.0, duration)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _lookup_start_pose(
    node: NavigateGoalClient,
    tf_buffer: Buffer,
    *,
    frame_id: str,
    robot_base_frame: str,
    timeout_sec: float,
) -> tuple[float, float, float]:
    deadline = time.monotonic() + timeout_sec
    last_error = "transform not received"
    while rclpy.ok() and time.monotonic() < deadline:
        try:
            transform = tf_buffer.lookup_transform(
                frame_id,
                robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return (
                float(translation.x),
                float(translation.y),
                _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            )
        except TransformException as exc:
            last_error = str(exc)
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(
        f"TF {frame_id} <- {robot_base_frame} unavailable after {timeout_sec:.1f}s: {last_error}"
    )


def _materialize_goals(
    goals: list[dict],
    *,
    coordinate_mode: str,
    origin: tuple[float, float, float] | None,
) -> list[dict]:
    if coordinate_mode == "absolute":
        return [dict(goal) for goal in goals]
    if coordinate_mode != "relative_to_start":
        raise ValueError(
            f"unsupported goal_coordinates={coordinate_mode!r}; expected absolute or relative_to_start"
        )
    if origin is None:
        raise ValueError("relative_to_start requires an origin pose")

    converted = []
    for goal in goals:
        x, y, yaw = compose_planar_pose(
            origin_x=origin[0],
            origin_y=origin[1],
            origin_yaw=origin[2],
            relative_x=float(goal["x"]),
            relative_y=float(goal["y"]),
            relative_yaw=float(goal["yaw"]),
        )
        row = dict(goal)
        row["relative_x"] = float(goal["x"])
        row["relative_y"] = float(goal["y"])
        row["relative_yaw"] = float(goal["yaw"])
        row["x"] = x
        row["y"] = y
        row["yaw"] = yaw
        converted.append(row)
    return converted


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    scenario = yaml.safe_load(cli.scenario.read_text(encoding="utf-8"))
    action = scenario.get("action_name", "/navigate_to_pose")
    frame = scenario.get("frame_id", "map")
    robot_base_frame = scenario.get("robot_base_frame", "leo1/base_link")
    coordinate_mode = scenario.get("goal_coordinates", "absolute")
    server_timeout = float(scenario.get("server_timeout_sec", 45.0))
    transform_timeout = float(scenario.get("transform_timeout_sec", 15.0))
    settle = float(scenario.get("settle_time_sec", 2.0))
    configured_goals = scenario.get("goals", [])
    acceptance = scenario.get("acceptance", {})
    if not configured_goals:
        print("ERROR: scenario contains no goals", file=sys.stderr)
        return 2

    rclpy.init(args=raw)
    node = NavigateGoalClient(action)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    results = []
    try:
        print(f"Waiting {cli.initial_wait:.1f}s for SLAM and Nav2 to settle...")
        _spin_wait(node, cli.initial_wait)
        if not node.wait_for_server(server_timeout):
            print(f"ERROR: action server {action} unavailable", file=sys.stderr)
            return 3

        origin = None
        if coordinate_mode == "relative_to_start":
            try:
                origin = _lookup_start_pose(
                    node,
                    tf_buffer,
                    frame_id=frame,
                    robot_base_frame=robot_base_frame,
                    timeout_sec=transform_timeout,
                )
            except RuntimeError as exc:
                print(f"ERROR: {exc}", file=sys.stderr)
                return 3
            print(
                "Captured start pose: "
                f"x={origin[0]:.3f}, y={origin[1]:.3f}, yaw={math.degrees(origin[2]):.1f} deg"
            )

        try:
            goals = _materialize_goals(
                configured_goals,
                coordinate_mode=coordinate_mode,
                origin=origin,
            )
        except (KeyError, TypeError, ValueError) as exc:
            print(f"ERROR: invalid scenario: {exc}", file=sys.stderr)
            return 2

        for index, goal in enumerate(goals, start=1):
            name = str(goal.get("name", f"goal_{index}"))
            relative = ""
            if coordinate_mode == "relative_to_start":
                relative = (
                    f" relative=({goal['relative_x']:.3f}, {goal['relative_y']:.3f}, "
                    f"{math.degrees(goal['relative_yaw']):.1f} deg)"
                )
            print(
                f"[{index}/{len(goals)}] {name}: map x={goal['x']:.3f}, y={goal['y']:.3f}, "
                f"yaw={math.degrees(float(goal['yaw'])):.1f} deg{relative}"
            )
            outcome = node.send_and_wait(
                x=float(goal["x"]),
                y=float(goal["y"]),
                yaw=float(goal["yaw"]),
                frame_id=frame,
                timeout_sec=float(goal.get("timeout_sec", 120.0)),
            )
            row = {
                "name": name,
                "accepted": outcome.accepted,
                "status": outcome.status,
                "result": outcome.message,
                "elapsed_sec": round(outcome.elapsed_sec, 3),
                "succeeded": outcome.succeeded,
                "goal_map": {
                    "x": round(float(goal["x"]), 6),
                    "y": round(float(goal["y"]), 6),
                    "yaw": round(float(goal["yaw"]), 6),
                },
            }
            results.append(row)
            print(json.dumps(row, sort_keys=True))
            if not outcome.succeeded and not cli.continue_on_failure:
                break
            _spin_wait(node, settle)

        success_count = sum(1 for row in results if row["succeeded"])
        failure_count = sum(1 for row in results if not row["succeeded"])
        required = int(acceptance.get("required_successes", len(goals)))
        max_failures = int(acceptance.get("max_failures", 0))
        passed = success_count >= required and failure_count <= max_failures and len(results) == len(goals)
        summary = {
            "scenario": str(cli.scenario),
            "goal_coordinates": coordinate_mode,
            "captured_start_pose": (
                None
                if origin is None
                else {"x": origin[0], "y": origin[1], "yaw": origin[2]}
            ),
            "goals_planned": len(goals),
            "goals_executed": len(results),
            "successes": success_count,
            "failures": failure_count,
            "required_successes": required,
            "max_failures": max_failures,
            "passed_action_regression": passed,
            "results": results,
            "manual_checks_still_required": [
                "zero physical/model contacts",
                "no duplicate-room map deformation",
                "no second final cmd_vel publisher",
            ],
        }
        rendered = json.dumps(summary, indent=2, sort_keys=True)
        print(rendered)
        if cli.output:
            cli.output.parent.mkdir(parents=True, exist_ok=True)
            cli.output.write_text(rendered + "\n", encoding="utf-8")
        return 0 if passed else 4
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
