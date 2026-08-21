#!/usr/bin/env python3
"""Record map, costmaps, plan, pose, frontiers, nav2 goal status and a
low-res camera feed for offline run replay.

Usage: run_recorder.py [out_dir]

Writes <out>/frame_<n>.npz every ~2 s (int8 grids, one Path, one pose,
frontier points), <out>/video/img_<t>.jpg at ~1 Hz (320x180 JPEG), and
<out>/events.jsonl with every NavigateToPose goal status transition.
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "leo_nav2_ws/runs/latest"
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "video").mkdir(exist_ok=True)

STATUS_NAMES = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


class Recorder(Node):
    def __init__(self):
        super().__init__("run_recorder")
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map = None
        self.gcost = None
        self.lcost = None
        self.plan = None
        self.frontier_pts = None
        self.frontier_centroids = None
        self.image = None
        self.goal_states = {}
        self.events = open(OUT / "events.jsonl", "a", buffering=1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self._m, latched)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._g, latched)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._l, latched)
        self.create_subscription(NavPath, "/plan", self._p, 5)
        self.create_subscription(MarkerArray, "/explore/frontiers", self._f, 5)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._s, latched
        )
        self.create_subscription(
            Image, "/rob_4/camera/color/image_raw", self._i, qos_profile_sensor_data
        )
        self.n = 0
        self.create_timer(2.0, self.snap)
        self.create_timer(1.0, self.save_image)
        self._log_event({"event": "recorder_start"})
        self.get_logger().info(f"recording to {OUT}")

    def _log_event(self, d):
        d["t"] = time.time()
        self.events.write(json.dumps(d) + "\n")

    @staticmethod
    def _pack(msg):
        return (
            np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
            np.array([msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution]),
        )

    def _m(self, msg):
        self.map = self._pack(msg)

    def _g(self, msg):
        self.gcost = self._pack(msg)

    def _l(self, msg):
        self.lcost = self._pack(msg)

    def _p(self, msg):
        self.plan = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=np.float32
        )

    def _f(self, msg):
        pts, centroids = [], []
        for m in msg.markers:
            if m.points:  # POINTS markers carry the frontier cells
                pts.extend((p.x, p.y) for p in m.points)
            elif m.type == 2:  # SPHERE markers mark frontier centroids
                centroids.append((m.pose.position.x, m.pose.position.y))
        self.frontier_pts = np.array(pts, dtype=np.float32) if pts else np.zeros((0, 2), np.float32)
        self.frontier_centroids = (
            np.array(centroids, dtype=np.float32) if centroids else np.zeros((0, 2), np.float32)
        )

    def _s(self, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()[:8]
            name = STATUS_NAMES.get(st.status, str(st.status))
            if self.goal_states.get(gid) != name:
                self.goal_states[gid] = name
                self._log_event({"event": "goal_status", "goal": gid, "status": name})

    def _i(self, msg):
        self.image = msg

    def save_image(self):
        msg, self.image = self.image, None
        if msg is None or msg.encoding not in ("rgb8", "bgr8"):
            return
        try:
            arr = np.frombuffer(msg.data, np.uint8)[: msg.height * msg.width * 3].reshape(
                msg.height, msg.width, 3
            )
            if msg.encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            small = cv2.resize(arr, (320, 180), interpolation=cv2.INTER_AREA)
            cv2.imwrite(
                str(OUT / "video" / f"img_{time.time():.1f}.jpg"),
                small,
                [cv2.IMWRITE_JPEG_QUALITY, 70],
            )
        except Exception as e:  # never let the video path kill grid recording
            self.get_logger().warn(f"image save failed: {e}", throttle_duration_sec=30)

    def snap(self):
        if self.map is None and self.gcost is None:
            return
        pose = np.array([np.nan, np.nan, np.nan])
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pose = np.array([t.transform.translation.x, t.transform.translation.y, yaw])
        except Exception:
            pass
        data = {"t": np.array([time.time()]), "pose": pose}
        for name, item in (("map", self.map), ("gcost", self.gcost), ("lcost", self.lcost)):
            if item is not None:
                data[name] = item[0]
                data[name + "_meta"] = item[1]
        if self.plan is not None:
            data["plan"] = self.plan
            if len(self.plan):
                data["goal"] = self.plan[-1]
        if self.frontier_pts is not None:
            data["frontier_pts"] = self.frontier_pts
            data["frontier_centroids"] = self.frontier_centroids
        np.savez_compressed(OUT / f"frame_{self.n:05d}.npz", **data)
        self.n += 1


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._log_event({"event": "recorder_stop"})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
