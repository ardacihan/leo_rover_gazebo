#!/usr/bin/env python3

"""Verify fail-closed gate behavior on an isolated ROS domain."""

import math
import os
import signal
import statistics
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


PREFIX = "/codex_gate"


class SafetyInputs(Node):
    def __init__(self):
        super().__init__("safety_gate_probe_inputs")
        self.publish_camera = True
        self.request_linear = 0.08
        self.request_pub = self.create_publisher(Twist, f"{PREFIX}/request", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, f"{PREFIX}/scan", qos_profile_sensor_data
        )
        self.camera_pub = self.create_publisher(
            LaserScan, f"{PREFIX}/camera", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, f"{PREFIX}/odom", qos_profile_sensor_data
        )
        self.battery_pub = self.create_publisher(
            Float32, f"{PREFIX}/battery", qos_profile_sensor_data
        )
        self.create_timer(0.05, self._publish)

    def _scan(self):
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_footprint"
        message.angle_min = -math.pi
        message.angle_increment = math.radians(1.0)
        message.angle_max = math.pi - message.angle_increment
        message.range_min = 0.15
        message.range_max = 3.0
        message.ranges = [2.0] * 360
        return message

    def _publish(self):
        command = Twist()
        command.linear.x = self.request_linear
        self.request_pub.publish(command)
        scan = self._scan()
        self.scan_pub.publish(scan)
        if self.publish_camera:
            self.camera_pub.publish(scan)
        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        voltage = Float32()
        voltage.data = 11.59
        self.battery_pub.publish(voltage)


class CollisionMonitorMock(Node):
    def __init__(self):
        super().__init__("collision_monitor")
        self.outputs = []
        self.output_pub = self.create_publisher(Twist, f"{PREFIX}/final", 10)
        self.create_subscription(Twist, f"{PREFIX}/raw", self._raw_callback, 10)

    def _raw_callback(self, message):
        self.outputs.append(float(message.linear.x))
        self.output_pub.publish(message)


class SupervisorMock(Node):
    def __init__(self):
        super().__init__("robot_supervisor_rgb")
        self.declare_parameter("enabled", False)
        # The real disabled supervisor retains this endpoint without publishing.
        self.output_pub = self.create_publisher(Twist, f"{PREFIX}/final", 10)


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


def main():
    process_log = tempfile.TemporaryFile(mode="w+")
    gate = subprocess.Popen(
        [
            "ros2", "run", "leo_rover_real_bringup", "safety_command_gate.py",
            "--ros-args",
            "-p", f"scan_topic:={PREFIX}/scan",
            "-p", f"camera_scan_topic:={PREFIX}/camera",
            "-p", f"odom_topic:={PREFIX}/odom",
            "-p", f"battery_topic:={PREFIX}/battery",
            "-p", f"cmd_vel_request_topic:={PREFIX}/request",
            "-p", f"cmd_vel_raw_topic:={PREFIX}/raw",
            "-p", f"cmd_vel_output_topic:={PREFIX}/final",
            "-p", "publish_filtered_scan:=false",
            "-p", "scan_yaw_offset:=0.0",
            "-p", "maximum_reverse_speed:=0.0",
            "-p", "allowed_cmd_vel_output_publishers:="
            "['collision_monitor','robot_supervisor_rgb']",
        ],
        stdout=process_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    inputs = monitor = supervisor = rogue = None
    executor = None
    passed = False
    try:
        rclpy.init()
        executor = SingleThreadedExecutor()
        inputs = SafetyInputs()
        monitor = CollisionMonitorMock()
        supervisor = SupervisorMock()
        executor.add_node(inputs)
        executor.add_node(monitor)
        executor.add_node(supervisor)
        healthy_deadline = time.monotonic() + 8.0
        healthy = []
        while time.monotonic() < healthy_deadline:
            spin_for(executor, 0.25)
            healthy = monitor.outputs[-10:]
            if healthy and statistics.median(healthy) >= 0.075:
                break
        if not healthy or statistics.median(healthy) < 0.075:
            raise RuntimeError(f"healthy command did not pass: {healthy}")

        inputs.publish_camera = False
        spin_for(executor, 1.0)
        stale_camera = monitor.outputs[-5:]
        if not stale_camera or max(abs(value) for value in stale_camera) > 0.001:
            raise RuntimeError(f"stale camera did not close gate: {stale_camera}")

        inputs.publish_camera = True
        spin_for(executor, 0.8)
        supervisor.set_parameters([
            Parameter("enabled", Parameter.Type.BOOL, True)
        ])
        spin_for(executor, 1.0)
        enabled_supervisor = monitor.outputs[-5:]
        if (
            not enabled_supervisor
            or max(abs(value) for value in enabled_supervisor) > 0.001
        ):
            raise RuntimeError(
                "enabled supervisor did not close gate: "
                f"{enabled_supervisor}"
            )

        supervisor.set_parameters([
            Parameter("enabled", Parameter.Type.BOOL, False)
        ])
        spin_for(executor, 1.0)
        restored = monitor.outputs[-5:]
        if not restored or statistics.median(restored) < 0.075:
            raise RuntimeError(
                f"gate did not reopen after supervisor disabled: {restored}"
            )
        rogue = Node("rogue_driver")
        rogue.create_publisher(Twist, f"{PREFIX}/final", 10)
        executor.add_node(rogue)
        spin_for(executor, 0.8)
        rogue_output = monitor.outputs[-5:]
        if not rogue_output or max(abs(value) for value in rogue_output) > 0.001:
            raise RuntimeError(f"rogue publisher did not close gate: {rogue_output}")

        passed = True
        print({
            "healthy_output_median": round(statistics.median(healthy), 3),
            "stale_camera_output": round(max(abs(value) for value in stale_camera), 3),
            "enabled_supervisor_output": round(
                max(abs(value) for value in enabled_supervisor), 3
            ),
            "restored_output_median": round(statistics.median(restored), 3),
            "rogue_publisher_output": round(max(abs(value) for value in rogue_output), 3),
            "passed": True,
        }, flush=True)
    finally:
        for node in (rogue, supervisor, monitor, inputs):
            if node is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass
                node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(gate)
        if not passed:
            process_log.seek(0)
            print(process_log.read(), flush=True)
        process_log.close()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
