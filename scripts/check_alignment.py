#!/usr/bin/env python3
"""Diagnose per-robot map/pose misalignment.

For each robot, looks up map->leo{i}/base_link and reports the merged /map
occupancy in a window around that cell. If a robot is sitting on/near
OCCUPIED cells, its live pose is misaligned with the merged map (which blocks
its planner with "legal potential" errors).

Usage (inside container): python3 check_alignment.py [robots-csv]
"""

import sys

import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener


class Check(Node):
    def __init__(self, robots):
        super().__init__('align_check')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.robots = robots
        self.msg = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        self.create_timer(2.0, self._check)
        self.n = 0

    def _cb(self, msg):
        self.msg = msg

    def _check(self):
        self.n += 1
        if self.msg is None:
            print('no merged map yet', flush=True)
            if self.n > 15:
                rclpy.shutdown()
            return
        info = self.msg.info
        grid = np.asarray(self.msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        for r in self.robots:
            try:
                tf = self.buf.lookup_transform(
                    'map', f'{r}/base_link', rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
            except Exception as e:
                print(f'{r}: no TF ({e})', flush=True)
                continue
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            cx = int((x - ox) / res)
            cy = int((y - oy) / res)
            k = 3
            occ = free = unk = 0
            for dy in range(-k, k + 1):
                for dx in range(-k, k + 1):
                    rr, cc = cy + dy, cx + dx
                    if 0 <= rr < info.height and 0 <= cc < info.width:
                        v = grid[rr, cc]
                        if v < 0:
                            unk += 1
                        elif v >= 50:
                            occ += 1
                        else:
                            free += 1
            here = grid[cy, cx] if (0 <= cy < info.height
                                    and 0 <= cx < info.width) else 'OOB'
            flag = '  <-- ON/NEAR OBSTACLE' if occ > 0 else ''
            print(f'{r}: pos=({x:.2f},{y:.2f}) cell=({cx},{cy}) '
                  f'val={here} window[occ={occ} free={free} unk={unk}]{flag}',
                  flush=True)
        print('---', flush=True)
        if self.n > 20:
            rclpy.shutdown()


def main():
    robots = sys.argv[1].split(',') if len(sys.argv) > 1 else ['leo1', 'leo2']
    rclpy.init()
    node = Check(robots)
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
