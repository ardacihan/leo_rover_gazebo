#!/usr/bin/env python3

"""Exercise explorer recovery using only /codex_probe/* ROS topics."""

import argparse
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


SCRIPTS = pathlib.Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from safe_room_explorer import ExplorationComplete, SafeRoomExplorer  # noqa: E402


class SyntheticSafetyChain(Node):
    """Publish synthetic safety inputs and echo or block requested commands."""

    def __init__(self, scenario):
        super().__init__(
            "synthetic_safety_chain",
            cli_args=[],
            use_global_arguments=False,
        )
        self.scenario = scenario
        self.started = time.monotonic()
        self.forward_seen_at = None
        self.reverse_seen_at = None
        self.boxed_front = False
        self.force_monitor_block = False
        self.saw_forward = False
        self.saw_turn = False
        self.saw_reverse = False
        self.saw_zero_after_reverse = False
        self.reverse_was_straight = True

        self.scan_pub = self.create_publisher(
            LaserScan, "/codex_probe/scan", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/codex_probe/odom", qos_profile_sensor_data
        )
        self.battery_pub = self.create_publisher(
            Float32, "/codex_probe/battery", qos_profile_sensor_data
        )
        self.output_pub = self.create_publisher(
            Twist, "/codex_probe/cmd_output", 10
        )
        self.create_subscription(
            Twist, "/codex_probe/cmd_request", self._command_callback, 10
        )
        self.create_timer(0.05, self._publish_inputs)

    def _command_callback(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        now = time.monotonic()
        if linear > 0.02:
            self.saw_forward = True
            if self.forward_seen_at is None:
                self.forward_seen_at = now
        if abs(angular) > 0.05:
            self.saw_turn = True
        if linear < -0.02:
            self.saw_reverse = True
            self.reverse_was_straight &= abs(angular) <= 0.01
            if self.reverse_seen_at is None:
                self.reverse_seen_at = now
        if self.saw_reverse and abs(linear) < 0.005 and abs(angular) < 0.01:
            self.saw_zero_after_reverse = True

        output = Twist()
        if not (self.force_monitor_block and not self.saw_reverse):
            output.linear.x = linear
            output.angular.z = angular
        self.output_pub.publish(output)

    def _publish_inputs(self):
        now = time.monotonic()
        if self.forward_seen_at is not None and now - self.forward_seen_at > 0.50:
            if self.scenario == "boxed":
                self.boxed_front = True
            else:
                self.force_monitor_block = True

        rear_blocked = (
            self.reverse_seen_at is not None and now - self.reverse_seen_at > 0.40
        )
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - math.radians(1.0)
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.02
        scan.range_max = 10.0
        scan.ranges = []
        for index in range(360):
            degrees = -180.0 + index
            distance = 2.0
            if self.boxed_front and abs(degrees) < 145.0:
                distance = 0.35
            if rear_blocked and abs(degrees) >= 145.0:
                distance = 0.40
            scan.ranges.append(distance)
        self.scan_pub.publish(scan)

        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        battery = Float32()
        battery.data = 12.2
        self.battery_pub.publish(battery)

    def passed(self):
        expected_turn = self.scenario == "monitor_block"
        return (
            self.saw_forward
            and self.saw_reverse
            and self.saw_zero_after_reverse
            and self.reverse_was_straight
            and (self.saw_turn or not expected_turn)
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=("boxed", "monitor_block"))
    parsed = parser.parse_args()

    rclpy.init(
        args=[
            "--ros-args",
            "-p", "scan_topic:=/codex_probe/scan",
            "-p", "odom_topic:=/codex_probe/odom",
            "-p", "battery_topic:=/codex_probe/battery",
            "-p", "cmd_vel_request_topic:=/codex_probe/cmd_request",
            "-p", "cmd_vel_output_topic:=/codex_probe/cmd_output",
            "-p", "run_duration:=12.0",
            "-p", "max_distance:=12.0",
            "-p", "planned_turn_distance:=0.0",
        ]
    )
    explorer = SafeRoomExplorer()
    chain = SyntheticSafetyChain(parsed.scenario)
    executor = SingleThreadedExecutor()
    executor.add_node(explorer)
    executor.add_node(chain)
    deadline = time.monotonic() + 12.0
    try:
        while time.monotonic() < deadline and not chain.passed():
            try:
                executor.spin_once(timeout_sec=0.10)
            except ExplorationComplete:
                break
    finally:
        result = {
            "scenario": parsed.scenario,
            "forward": chain.saw_forward,
            "turn": chain.saw_turn,
            "reverse": chain.saw_reverse,
            "straight_reverse": chain.reverse_was_straight,
            "stopped_after_reverse": chain.saw_zero_after_reverse,
        }
        print(result, flush=True)
        passed = chain.passed()
        executor.remove_node(chain)
        executor.remove_node(explorer)
        chain.destroy_node()
        explorer.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
