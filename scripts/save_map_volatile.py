#!/usr/bin/env python3
"""Save an OccupancyGrid to .pgm/.yaml from a VOLATILE publisher.

`nav2_map_server map_saver_cli` subscribes with TRANSIENT_LOCAL durability,
which is *incompatible* with a VOLATILE publisher -- rmw simply never matches
them and the saver times out with "Failed to spin map subscription".
`shared_map_merger` publishes /shared_map VOLATILE, so the merged map, the one
artifact the whole two-rover pipeline exists to produce, cannot be saved by the
standard tool at all. This subscribes VOLATILE and writes the same
map_server-format pair.

Usage: save_map_volatile.py <topic> <out_stem> [timeout_sec]
"""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class Saver(Node):
    def __init__(self, topic, stem):
        super().__init__('save_map_volatile')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.stem = stem
        self.done = False
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, topic, self._cb, qos)
        self.get_logger().info(f'waiting for {topic} (VOLATILE)')

    def _cb(self, msg):
        if self.done:
            return
        info = msg.info
        g = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        # map_server PGM convention: 254 free, 0 occupied, 205 unknown,
        # row 0 of the file is the TOP row (max y).
        img = np.full(g.shape, 205, dtype=np.uint8)
        img[(g >= 0) & (g < 50)] = 254
        img[g >= 50] = 0
        img = np.flipud(img)
        with open(self.stem + '.pgm', 'wb') as fh:
            fh.write(b'P5\n')
            fh.write(f'{info.width} {info.height}\n255\n'.encode())
            fh.write(img.tobytes())
        o = info.origin.position
        with open(self.stem + '.yaml', 'w') as fh:
            fh.write(f'image: {self.stem.split("/")[-1]}.pgm\n'
                     f'resolution: {info.resolution}\n'
                     f'origin: [{o.x}, {o.y}, 0.0]\n'
                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        self.get_logger().info(
            f'saved {self.stem}.pgm ({info.width}x{info.height} @ '
            f'{info.resolution} m, origin {o.x:.2f} {o.y:.2f})')
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/shared_map'
    stem = sys.argv[2] if len(sys.argv) > 2 else '/ros2_ws/shared_map'
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
    rclpy.init()
    node = Saver(topic, stem)
    deadline = time.time() + timeout
    while rclpy.ok() and not node.done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    ok = node.done
    if not ok:
        node.get_logger().error(f'no message on {topic} within {timeout}s')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
