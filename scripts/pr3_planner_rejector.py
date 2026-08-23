#!/usr/bin/env python3
"""Fault-injection action server for PR3 escape verification.

The server returns an empty path for the first N requests, then returns a
minimal non-empty path. Remap the explorer's compute_path_to_pose client to
this server; NavigateToPose remains connected to real Nav2.
"""

import sys

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PlannerRejector(Node):

    def __init__(self, reject_count):
        super().__init__('pr3_planner_rejector')
        self.reject_count = reject_count
        self.requests = 0
        self.server = ActionServer(
            self,
            ComputePathToPose,
            '/pr3_compute_path_to_pose',
            self.execute,
        )
        self.get_logger().info(
            f'Rejecting the first {self.reject_count} path validations')

    def execute(self, goal_handle):
        self.requests += 1
        result = ComputePathToPose.Result()
        result.path = Path()
        result.path.header = goal_handle.request.goal.header

        if self.requests <= self.reject_count:
            self.get_logger().warn(
                f'Injected planner rejection {self.requests}/'
                f'{self.reject_count}')
        else:
            result.path.poses.append(goal_handle.request.goal)
            self.get_logger().info(
                f'Validation {self.requests} passed to real navigation')

        goal_handle.succeed()
        return result


def main():
    reject_count = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rclpy.init()
    node = PlannerRejector(reject_count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
