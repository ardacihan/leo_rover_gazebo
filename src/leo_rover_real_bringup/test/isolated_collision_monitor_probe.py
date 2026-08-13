#!/usr/bin/env python3

"""Verify directional Collision Monitor behavior on an isolated ROS domain."""

import argparse
import math
import os
import pathlib
import signal
import statistics
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


SCAN_TOPIC = "/codex_cm/scan"
INPUT_TOPIC = "/codex_cm/raw"
OUTPUT_TOPIC = "/codex_cm/output"


class CollisionProbe(Node):
    def __init__(self):
        super().__init__(
            "collision_probe", cli_args=[], use_global_arguments=False
        )
        self.scan_pub = self.create_publisher(
            LaserScan, SCAN_TOPIC, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, INPUT_TOPIC, 10)
        self.create_subscription(Twist, OUTPUT_TOPIC, self._output_callback, 10)
        self.phase = "waiting"
        self.forward_outputs = []
        self.reverse_outputs = []
        self.create_timer(0.05, self._publish)

    def _output_callback(self, msg):
        if self.phase == "forward":
            self.forward_outputs.append(float(msg.linear.x))
        elif self.phase == "reverse":
            self.reverse_outputs.append(float(msg.linear.x))

    def _publish(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "probe_laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - math.radians(1.0)
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.02
        scan.range_max = 10.0
        scan.ranges = [10.0] * 360
        # Outside the 0.35 m protected body, but inside the forward 1.5 s
        # approach envelope at 0.08 m/s. Reverse should remain unconstrained.
        for index in range(174, 187):
            scan.ranges[index] = 0.42
        self.scan_pub.publish(scan)

        cmd = Twist()
        if self.phase == "forward":
            cmd.linear.x = 0.08
        elif self.phase == "reverse":
            cmd.linear.x = -0.04
        self.cmd_pub.publish(cmd)

    def publish_zeros(self):
        self.phase = "waiting"
        zero = Twist()
        for _ in range(20):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)


def run_phase(node, phase, duration):
    node.phase = phase
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=pathlib.Path)
    parsed = parser.parse_args()
    if not str(parsed.config.resolve()).endswith("collision_monitor_params.yaml"):
        raise SystemExit("refusing unexpected Collision Monitor config")

    process_log = tempfile.TemporaryFile(mode="w+")
    static_tf = subprocess.Popen(
        [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "probe_base", "--child-frame-id", "probe_laser",
        ],
        stdout=process_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    monitor = subprocess.Popen(
        [
            "ros2", "run", "nav2_collision_monitor", "collision_monitor",
            "--ros-args", "--params-file", str(parsed.config.resolve()),
            "-p", "base_frame_id:=probe_base",
            "-p", "odom_frame_id:=probe_odom",
            "-p", "base_shift_correction:=false",
            "-p", f"cmd_vel_in_topic:={INPUT_TOPIC}",
            "-p", f"cmd_vel_out_topic:={OUTPUT_TOPIC}",
            "-p", f"fused_scan.topic:={SCAN_TOPIC}",
        ],
        stdout=process_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    node = None
    passed = False
    try:
        time.sleep(0.5)
        for transition in ("configure", "activate"):
            transition_deadline = time.monotonic() + 12.0
            result = None
            while time.monotonic() < transition_deadline:
                result = subprocess.run(
                    ["ros2", "lifecycle", "set", "/collision_monitor", transition],
                    text=True,
                    capture_output=True,
                    timeout=5.0,
                )
                if result.returncode == 0 and "success" in result.stdout.lower():
                    break
                time.sleep(0.25)
            else:
                raise RuntimeError(
                    f"lifecycle {transition} failed: "
                    f"{result.stdout}{result.stderr}"
                )

        rclpy.init()
        node = CollisionProbe()
        discovery_deadline = time.monotonic() + 5.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
            if (
                node.cmd_pub.get_subscription_count() > 0
                and node.count_publishers(OUTPUT_TOPIC) > 0
            ):
                break
        else:
            raise RuntimeError("isolated Collision Monitor discovery timed out")

        # Discovery and lifecycle activation can be slow on the Jetson while
        # RealSense/SLAM are active in another isolated DDS domain. Give each
        # directional phase enough samples to avoid a scheduler-dependent test.
        run_phase(node, "waiting", 0.5)
        run_phase(node, "forward", 3.0)
        run_phase(node, "reverse", 3.0)
        forward = node.forward_outputs[-10:]
        reverse = node.reverse_outputs[-10:]
        if not forward or not reverse:
            raise RuntimeError(
                "missing Collision Monitor output samples: "
                f"forward={len(node.forward_outputs)}, "
                f"reverse={len(node.reverse_outputs)}"
            )
        forward_median = statistics.median(forward)
        reverse_median = statistics.median(reverse)
        passed = 0.0 <= forward_median < 0.06 and reverse_median <= -0.035
        print(
            {
                "forward_request": 0.08,
                "forward_output_median": round(forward_median, 4),
                "reverse_request": -0.04,
                "reverse_output_median": round(reverse_median, 4),
                "passed": passed,
            },
            flush=True,
        )
    finally:
        if node is not None:
            node.publish_zeros()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(monitor)
        stop_process(static_tf)
        if not passed:
            process_log.seek(0)
            print(process_log.read(), flush=True)
        process_log.close()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
