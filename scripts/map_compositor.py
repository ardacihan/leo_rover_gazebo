#!/usr/bin/env python3
"""Deterministic multi-robot map compositor (replaces multirobot_map_merge).

multirobot_map_merge's known-init-pose params (leading-slash names) do not take
effect under our Humble build, so the offset robot's submap is placed ~init
metres wrong, misaligning its costmap and blocking its planner. Here we
composite the per-robot SLAM maps ourselves, anchored at the SAME known
offsets as the static map->leo{i}/map transforms.

Since the report-ready-maps PR this is no longer a naive overwrite merge:

  * log-odds fusion  - per-cell sum of occupied/free votes instead of max();
    a wall seen by one robot survives, an occupied-vs-free disagreement
    (drift residue) decays to unknown instead of painting phantom walls.
  * drift registration - every --register-period seconds the offset robot's
    submap pose is re-refined against leo1's by correlative matching in a
    small window around the known offset (map_fusion.register), absorbing
    per-robot SLAM drift that fixed offsets cannot.
  * despeckle       - isolated occupied blobs < 6 cells are removed.

Publishes the merged OccupancyGrid on /map (frame `map`), transient-local
like a map server.

Usage: map_compositor.py [robots=leo1,leo2] [rate=2.0] [--naive] [--no-register]
"""

import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from map_fusion import GridMap, clean, fuse, register

# leo{i}/map frame position in the merged `map` frame (x, y). IDENTITY, not
# the spawn offsets: /leo{i}/odom is Gazebo's world-frame OdometryPublisher,
# so every robot's slam map frame is already world-anchored (see
# map_merge_leo.launch.py). MUST match the static TFs published there.
OFFSETS = {'leo1': (0.0, 0.0), 'leo2': (0.0, 0.0)}

REGISTER_PERIOD = 30.0     # s between drift re-registrations
REGISTER_WINDOW = (0.25, 0.25, math.radians(2.0))
MIN_OCC_CELLS = 500        # both maps need walls before registering


class Compositor(Node):
    def __init__(self, robots, rate, naive=False, do_register=True):
        super().__init__('map_compositor')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.robots = robots
        self.naive = naive
        self.do_register = do_register
        self.submaps = {}
        self.refined = {}          # robot -> (tx, ty, th) refined pose
        self.last_register = 0.0
        qos_in = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        for r in robots:
            self.create_subscription(
                OccupancyGrid, f'/{r}/map',
                lambda m, r=r: self.submaps.__setitem__(r, m), qos_in)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos_in)
        self.create_timer(1.0 / rate, self._merge)

    @staticmethod
    def _to_gridmap(msg, name):
        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        return GridMap(grid, (msg.info.origin.position.x,
                              msg.info.origin.position.y),
                       msg.info.resolution, name)

    def _pose_of(self, robot, gm):
        ox, oy = OFFSETS.get(robot, (0.0, 0.0))
        seed = (ox, oy, 0.0)
        if robot == self.robots[0] or not self.do_register:
            return self.refined.get(robot, seed)
        now = self.get_clock().now().nanoseconds / 1e9
        ref_msg = self.submaps.get(self.robots[0])
        if ref_msg is None:
            return self.refined.get(robot, seed)
        if now - self.last_register >= REGISTER_PERIOD:
            ref = self._to_gridmap(ref_msg, self.robots[0])
            if ((ref.grid == 100).sum() >= MIN_OCC_CELLS
                    and (gm.grid == 100).sum() >= MIN_OCC_CELLS):
                self.last_register = now
                start = self.refined.get(robot, seed)
                pose, score = register(gm, ref, start, (0.0, 0.0, 0.0),
                                       REGISTER_WINDOW, verbose=False)
                self.refined[robot] = pose
                self.get_logger().info(
                    f'registered {robot}: offset '
                    f'({pose[0]:.3f}, {pose[1]:.3f}, '
                    f'{math.degrees(pose[2]):.2f}deg) score={score:.3f}')
        return self.refined.get(robot, seed)

    def _merge(self):
        maps = {r: m for r, m in self.submaps.items() if m is not None}
        if not maps:
            return
        pairs = []
        for r, m in maps.items():
            gm = self._to_gridmap(m, r)
            pairs.append((gm, self._pose_of(r, gm)))
        try:
            merged, _ = fuse(pairs, mode='naive' if self.naive else 'logodds')
            if not self.naive:
                merged = clean(merged, bounds=None, min_occ_cells=6,
                               max_unk_hole=0)
        except (ValueError, MemoryError) as e:
            self.get_logger().warn(f'merge failed: {e}')
            return
        H, W = merged.shape
        if W * H > 8_000_000:
            return
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = merged.res
        out.info.width = W
        out.info.height = H
        out.info.origin.position.x = merged.origin[0]
        out.info.origin.position.y = merged.origin[1]
        out.info.origin.orientation.w = 1.0
        out.data = merged.grid.astype(np.int8).flatten().tolist()
        self.pub.publish(out)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    flags = {a for a in sys.argv[1:] if a.startswith('--')}
    robots = args[0].split(',') if args else ['leo1', 'leo2']
    rate = float(args[1]) if len(args) > 1 else 2.0
    rclpy.init()
    node = Compositor(robots, rate, naive='--naive' in flags,
                      do_register='--no-register' not in flags)
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
