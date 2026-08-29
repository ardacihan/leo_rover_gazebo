#!/usr/bin/env python3
"""Frame-by-frame capture of map, costmaps, plan and frontiers -- alignable.

Replaces the recorder that produced runs/*/frame_*.npz on 2026-08-25. That
one stored each grid as (origin_x, origin_y, resolution) and nothing else,
which threw away the two things needed to place a grid in the world:

  * header.frame_id  -- /local_costmap/costmap is in `odom`, /map and
    /global_costmap/costmap are in `map`. Without the frame you cannot know
    they are different, and drawing them together silently misregisters the
    local costmap by however far map->odom has drifted (0.3-1.0 m on
    2026-08-25 run 2, growing through the run).
  * info.origin.orientation -- a grid whose origin carries a yaw is drawn
    rotated. Dropping it cannot be recovered afterwards.

This node stores both, plus the live map->odom, so every layer can be put in
one frame offline. It also stamps how stale each grid was when captured, so a
frozen /map is visible in the data instead of looking like a slow one.

  ros2 run leo_nav2_exploration grid_recorder --ros-args \
      -p out_dir:=$HOME/bags/run7/frames -p period:=2.0

or standalone:  OUT_DIR=... python3 grid_recorder.py
"""
from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener


def latched(depth: int = 1) -> QoSProfile:
    """Costmaps and /map are TRANSIENT_LOCAL. A VOLATILE subscriber gets no
    latched backlog and, if it connects late, may get nothing at all."""
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q) -> float:
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z)))


def unpack(msg: OccupancyGrid):
    """-> (grid int8 [h,w], meta float64 [ox, oy, res, yaw], frame_id, stamp)"""
    g = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height,
                                                    msg.info.width)
    meta = np.array([msg.info.origin.position.x,
                     msg.info.origin.position.y,
                     msg.info.resolution,
                     yaw_of(msg.info.origin.orientation)], dtype=np.float64)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return g, meta, msg.header.frame_id, stamp


class GridRecorder(Node):
    def __init__(self):
        super().__init__("grid_recorder")
        self.declare_parameter("out_dir",
                               os.environ.get("OUT_DIR", "~/bags/frames"))
        self.declare_parameter("period", float(os.environ.get("PERIOD", "2.0")))
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        P = lambda n: self.get_parameter(n).value
        self.out = os.path.expanduser(P("out_dir"))
        os.makedirs(self.out, exist_ok=True)
        self.map_frame, self.odom_frame = P("map_frame"), P("odom_frame")
        self.base_frame = P("base_frame")

        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self)

        self.latest = {}          # key -> (grid, meta, frame_id, stamp, rx_time)
        self.plan = None
        self.goal = None

        self.create_subscription(OccupancyGrid, P("map_topic"),
                                 lambda m: self._grid("map", m), latched())
        self.create_subscription(OccupancyGrid, P("global_costmap_topic"),
                                 lambda m: self._grid("gcost", m), latched())
        self.create_subscription(OccupancyGrid, P("local_costmap_topic"),
                                 lambda m: self._grid("lcost", m), latched())
        self.create_subscription(Path, P("plan_topic"), self._on_plan, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)

        self.n = 0
        self.create_timer(float(P("period")), self.tick)
        self.get_logger().info(f"grid_recorder -> {self.out} "
                               f"every {P('period')} s")

    # ---- callbacks -------------------------------------------------------
    def _grid(self, key, msg):
        g, meta, frame, stamp = unpack(msg)
        prev = self.latest.get(key)
        # changed? compare content, not arrival -- a republished identical grid
        # is a frozen map, and that is exactly what we want to be able to see.
        changed = prev is None or not np.array_equal(prev[0], g) \
            or not np.array_equal(prev[1], meta)
        last_change = time.time() if changed else prev[5]
        self.latest[key] = (g, meta, frame, stamp, time.time(), last_change)

    def _on_plan(self, msg):
        self.plan = np.array([[p.pose.position.x, p.pose.position.y]
                              for p in msg.poses], dtype=np.float32)

    def _on_goal(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y,
                              yaw_of(msg.pose.orientation)], dtype=np.float64)

    # ---- TF --------------------------------------------------------------
    def _lookup(self, target, source):
        try:
            tr = self.tfbuf.lookup_transform(target, source,
                                             rclpy.time.Time()).transform
            return np.array([tr.translation.x, tr.translation.y,
                             yaw_of(tr.rotation)], dtype=np.float64)
        except Exception:
            return np.array([np.nan, np.nan, np.nan])

    # ---- write -----------------------------------------------------------
    def tick(self):
        now = time.time()
        payload = {
            "t": np.array([now]),
            # THE fix: the transform that relates the two costmap frames.
            # Store it every frame; it changes whenever SLAM corrects.
            "map_to_odom": self._lookup(self.map_frame, self.odom_frame),
            "pose_map": self._lookup(self.map_frame, self.base_frame),
            "pose_odom": self._lookup(self.odom_frame, self.base_frame),
        }
        for key, val in self.latest.items():
            g, meta, frame, stamp, rx, last_change = val
            payload[key] = g
            payload[f"{key}_meta"] = meta                 # ox, oy, res, YAW
            payload[f"{key}_frame"] = np.array(frame)     # 'map' or 'odom'
            payload[f"{key}_stamp"] = np.array([stamp])
            payload[f"{key}_age"] = np.array([now - last_change])
        if self.plan is not None and len(self.plan):
            payload["plan"] = self.plan
        if self.goal is not None:
            payload["goal"] = self.goal

        np.savez_compressed(os.path.join(self.out, f"frame_{self.n:05d}.npz"),
                            **payload)
        # loud, once every 5 frames, so a frozen map is obvious on the console
        age = payload.get("map_age")
        if age is not None and float(age[0]) > 15.0 and self.n % 5 == 0:
            self.get_logger().error(
                f"/map frozen for {float(age[0]):.0f} s -- restart SLAM")
        mo = payload["map_to_odom"]
        if self.n == 30 and np.isfinite(mo).all() and abs(mo[0]) < 1e-9 \
                and abs(mo[1]) < 1e-9 and abs(mo[2]) < 1e-9:
            self.get_logger().warning(
                "map->odom still exactly identity after 30 frames: "
                "slam_toolbox may have no input (check /scan_uniform)")
        self.n += 1


def main():
    rclpy.init()
    node = GridRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"wrote {node.n} frames to {node.out}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
