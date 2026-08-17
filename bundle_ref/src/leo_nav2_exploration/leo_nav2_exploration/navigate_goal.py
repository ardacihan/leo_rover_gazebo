"""Small NavigateToPose action client used by manual and regression tests."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


@dataclass(frozen=True)
class NavigationOutcome:
    accepted: bool
    status: int
    elapsed_sec: float
    message: str

    @property
    def succeeded(self) -> bool:
        return self.accepted and self.status == GoalStatus.STATUS_SUCCEEDED


class NavigateGoalClient(Node):
    def __init__(self, action_name: str = "/navigate_to_pose") -> None:
        super().__init__("navigate_goal_client")
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._last_feedback_log = 0.0

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_and_wait(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        timeout_sec: float,
    ) -> NavigationOutcome:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        start = time.monotonic()
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        while rclpy.ok() and not send_future.done() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.10)
        if not send_future.done():
            return NavigationOutcome(False, GoalStatus.STATUS_UNKNOWN, time.monotonic() - start, "goal_send_timeout")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return NavigationOutcome(False, GoalStatus.STATUS_UNKNOWN, time.monotonic() - start, "goal_rejected")

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.10)
            if time.monotonic() - start > timeout_sec:
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done() and time.monotonic() - start < timeout_sec + 3.0:
                    rclpy.spin_once(self, timeout_sec=0.10)
                return NavigationOutcome(True, GoalStatus.STATUS_CANCELED, time.monotonic() - start, "goal_timeout_cancelled")
        if not result_future.done():
            return NavigationOutcome(True, GoalStatus.STATUS_UNKNOWN, time.monotonic() - start, "shutdown_before_result")
        wrapped = result_future.result()
        if wrapped is None:
            return NavigationOutcome(True, GoalStatus.STATUS_UNKNOWN, time.monotonic() - start, "empty_result")
        status = int(wrapped.status)
        names = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "cancelled",
        }
        return NavigationOutcome(True, status, time.monotonic() - start, names.get(status, f"status_{status}"))

    def _feedback(self, message) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        feedback = message.feedback
        distance = getattr(feedback, "distance_remaining", float("nan"))
        recoveries = getattr(feedback, "number_of_recoveries", -1)
        self.get_logger().info(
            f"distance_remaining={distance:.3f} m, recoveries={recoveries}"
        )
        self._last_feedback_log = now


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send one Nav2 NavigateToPose goal.")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw_deg", type=float)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--action", default="/navigate_to_pose")
    parser.add_argument("--server-timeout", type=float, default=30.0)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    return parser


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    rclpy.init(args=raw)
    node = NavigateGoalClient(cli.action)
    try:
        if not node.wait_for_server(cli.server_timeout):
            print(f"ERROR: action server {cli.action} unavailable", file=sys.stderr)
            return 2
        outcome = node.send_and_wait(
            x=cli.x,
            y=cli.y,
            yaw=math.radians(cli.yaw_deg),
            frame_id=cli.frame,
            timeout_sec=cli.goal_timeout,
        )
        print(
            f"accepted={outcome.accepted} status={outcome.status} result={outcome.message} "
            f"elapsed_sec={outcome.elapsed_sec:.2f}"
        )
        return 0 if outcome.succeeded else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
