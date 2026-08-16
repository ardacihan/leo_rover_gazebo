#!/usr/bin/env python3
"""Print SLAM map coverage (known area in m^2) periodically.

Usage (inside the container):
    python3 /ros2_ws/scripts/map_coverage.py [interval_sec]
"""

import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)


class CoverageMonitor(Node):
    def __init__(self, interval, bounds=None):
        super().__init__('map_coverage_monitor')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        # Optional (xmin, xmax, ymin, ymax) world-frame clip. Excludes phantom
        # cells the multi-robot compositor can place outside the real world
        # when a rover's odometry drifts, so coverage can't exceed the true
        # reachable area and all conditions are measured on the same footprint.
        self.bounds = bounds
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        self.create_timer(interval, self._report)
        self.msg = None

    def _cb(self, msg):
        self.msg = msg

    def _grid(self):
        info = self.msg.info
        g = np.asarray(self.msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        if self.bounds is not None:
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            xmin, xmax, ymin, ymax = self.bounds
            c0 = max(0, int((xmin - ox) / res))
            c1 = min(info.width, int((xmax - ox) / res) + 1)
            r0 = max(0, int((ymin - oy) / res))
            r1 = min(info.height, int((ymax - oy) / res) + 1)
            g = g[r0:r1, c0:c1]
        return g

    def _report(self):
        if self.msg is None:
            print('coverage: no map yet', flush=True)
            return
        grid = self._grid()
        known = int((grid >= 0).sum())
        free = int(((grid >= 0) & (grid < 50)).sum())
        occ = int((grid >= 50).sum())
        res = self.msg.info.resolution
        area = known * res * res
        t = self.get_clock().now().nanoseconds / 1e9
        print(f'coverage: t={t:.0f}s known={area:.1f}m2 '
              f'free={free * res * res:.1f}m2 occ={occ * res * res:.1f}m2 '
              f'({self.msg.info.width}x{self.msg.info.height})', flush=True)


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    bounds = None
    if len(sys.argv) > 5:
        bounds = tuple(float(a) for a in sys.argv[2:6])  # xmin xmax ymin ymax
    rclpy.init()
    node = CoverageMonitor(interval, bounds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
