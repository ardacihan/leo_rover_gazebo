"""Compare a rover's SLAM map against its global costmap.

Counts cells the costmap calls lethal that the SLAM map does not call
occupied -- i.e. obstacles that came from somewhere other than the map.
"""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

LETHAL = 253


class Probe(Node):
    def __init__(self, ns):
        super().__init__('costmap_probe')
        self.grids = {}
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, f'/{ns}/map',
                                 lambda m: self._cb('map', m), qos)
        self.create_subscription(OccupancyGrid, f'/{ns}/global_costmap/costmap',
                                 lambda m: self._cb('costmap', m), qos)

    def _cb(self, key, msg):
        self.grids[key] = msg


def main():
    ns = sys.argv[1]
    rclpy.init()
    n = Probe(ns)
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.grids) == 2:
            break
    if len(n.grids) < 2:
        print(f'{ns}: only got {list(n.grids)}')
        return
    m, c = n.grids['map'], n.grids['costmap']
    ma = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    ca = np.asarray(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
    same = (m.info.width == c.info.width and m.info.height == c.info.height
            and abs(m.info.origin.position.x - c.info.origin.position.x) < 1e-6)
    print(f'{ns}: map {ma.shape} origin ({m.info.origin.position.x:.2f},'
          f'{m.info.origin.position.y:.2f})  costmap {ca.shape} '
          f'origin ({c.info.origin.position.x:.2f},{c.info.origin.position.y:.2f})'
          f'  aligned={same}')
    occ = ma >= 65
    unknown = ma < 0
    print(f'  slam: occupied {occ.sum()}  free {((ma >= 0) & (ma < 65)).sum()}'
          f'  unknown {unknown.sum()}')
    if not same:
        print('  grids not aligned; skipping the overlay comparison')
        return
    lethal = ca >= LETHAL
    print(f'  costmap: lethal {lethal.sum()}  >=200 {(ca >= 200).sum()}'
          f'  ==-1 {(ca < 0).sum()}')
    extra = lethal & ~occ & ~unknown
    extra_unknown = lethal & unknown
    print(f'  lethal on SLAM-free cells   : {extra.sum()}  '
          f'({100 * extra.sum() / max(lethal.sum(), 1):.1f}% of lethal)')
    print(f'  lethal on SLAM-unknown cells: {extra_unknown.sum()}')


main()
