#!/usr/bin/env python3

"""Exercise the mapping artifact recorder on an isolated ROS domain."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class MappingProbe(Node):
    def __init__(self):
        super().__init__("mapping_artifact_probe_driver")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, "/codex_mapping_probe/map", qos
        )
        self.broadcaster = TransformBroadcaster(self)
        self.client = self.create_client(
            Trigger, "/codex_mapping_recorder/save_mapping_artifacts"
        )
        self.start_time = time.monotonic()
        self.create_timer(0.05, self._publish)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        occupancy = OccupancyGrid()
        occupancy.header.stamp = stamp
        occupancy.header.frame_id = "map"
        occupancy.info.resolution = 0.10
        occupancy.info.width = 30
        occupancy.info.height = 20
        occupancy.info.origin.position.x = -1.0
        occupancy.info.origin.position.y = -1.0
        occupancy.info.origin.orientation.w = 1.0
        values = [0] * 600
        for x in range(30):
            values[x] = 100
            values[19 * 30 + x] = 100
        for y in range(20):
            values[y * 30] = 100
            values[y * 30 + 29] = 100
        occupancy.data = values
        self.map_publisher.publish(occupancy)

        elapsed = min(time.monotonic() - self.start_time, 2.0)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "probe_base"
        transform.transform.translation.x = -0.5 + 0.4 * elapsed
        transform.transform.translation.y = 0.1 * math.sin(elapsed)
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


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
    with tempfile.TemporaryDirectory(prefix="leo_mapping_probe_") as directory:
        log = tempfile.TemporaryFile(mode="w+")
        recorder = subprocess.Popen(
            [
                "ros2", "run", "leo_rover_real_bringup",
                "mapping_artifact_recorder.py", "--ros-args",
                "-r", "__ns:=/codex_mapping_recorder",
                "-p", "map_topic:=/codex_mapping_probe/map",
                "-p", "path_topic:=/codex_mapping_probe/path",
                "-p", "base_frame:=probe_base",
                "-p", "sample_period:=0.1",
                "-p", f"output_directory:={directory}",
                "-p", "artifact_prefix:=probe",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        node = None
        try:
            rclpy.init()
            node = MappingProbe()
            discovery_deadline = time.monotonic() + 5.0
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.client.service_is_ready():
                    break
            else:
                raise RuntimeError("recorder service discovery timed out")

            collection_deadline = time.monotonic() + 3.0
            while time.monotonic() < collection_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            future = node.client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done() or not future.result().success:
                message = "timeout" if not future.done() else future.result().message
                raise RuntimeError(f"save service failed: {message}")

            summaries = list(Path(directory).glob("*_summary.json"))
            overlays = list(Path(directory).glob("*_path.png"))
            csv_files = list(Path(directory).glob("*_path.csv"))
            maps = list(Path(directory).glob("*.pgm"))
            if not (len(summaries) == len(overlays) == len(csv_files) == len(maps) == 1):
                raise RuntimeError("expected artifact set was not created")
            summary = json.loads(summaries[0].read_text(encoding="utf-8"))
            if summary["path_samples"] < 5 or summary["path_length_m"] < 0.25:
                raise RuntimeError(f"recorded path is too short: {summary}")
            if not overlays[0].read_bytes().startswith(b"\x89PNG\r\n\x1a\n"):
                raise RuntimeError("overlay is not a PNG")
            print({
                "path_samples": summary["path_samples"],
                "path_length_m": round(summary["path_length_m"], 3),
                "known_cells": summary["known_cells"],
                "artifacts": 5,
                "passed": True,
            }, flush=True)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stop_process(recorder)
            if recorder.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                print(log.read(), flush=True)
            log.close()


if __name__ == "__main__":
    main()
