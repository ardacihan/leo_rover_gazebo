#!/usr/bin/env python3
"""Record both per-robot maps, the shared map, poses and the alignment over time.

`map_recorder.py` subscribes to a hardcoded `/map`, which does not exist in the
tag-aligned two-rover stack -- the whole 2026-08-23 session recorded empty
`exploration/` directories because of it. This records what actually exists,
and records it as *data* rather than as pre-rendered pictures, so the time-lapse
can be re-rendered offline any number of ways without re-running a 25-minute
simulation.

One compressed `.npz` per snapshot containing:

    leo1, leo2, shared   the three occupancy grids (int8)
    *_info               origin_x, origin_y, resolution for each
    p1, p2               robot poses in their own map frames
    tf                   the alignment transform in use (x, y, yaw), NaN if none
    locked               whether the bridge had published a transform
    t                    sim time

Usage (inside the container):
    merge_timelapse_recorder.py <out_dir> [interval_sec]
"""

import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray
import json
import math


def _grid(msg):
    if msg is None:
        return None, (0.0, 0.0, 0.05)
    g = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return g, (msg.info.origin.position.x, msg.info.origin.position.y,
               msg.info.resolution)


class MergeTimelapse(Node):
    def __init__(self, out_dir, interval):
        super().__init__('merge_timelapse_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.n = 0
        self.maps = {'leo1': None, 'leo2': None, 'shared': None}
        self.tf_est = None
        self.locked = False
        # Everything below exists so the recording can answer "why did it do
        # that", not just "what did the map look like": which frontier each
        # rover picked and why it was navigating, which tags it had seen, and
        # how confident the alignment was at that instant.
        self.status = {'leo1': {}, 'leo2': {}}
        self.frontiers = {'leo1': [], 'leo2': []}
        self.tags_seen = {'leo1': {}, 'leo2': {}}
        self.tag_events = []          # (t, robot, id) first sighting only
        self.conf = {'tag': float('nan'), 'map': float('nan')}

        # VOLATILE matches BOTH a VOLATILE publisher (shared_map_merger) and a
        # TRANSIENT_LOCAL one (slam_toolbox). A TRANSIENT_LOCAL subscriber
        # matches only the latter, and fails silently against the former.
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/leo1/map',
                                 lambda m: self._set('leo1', m), qos)
        self.create_subscription(OccupancyGrid, '/leo2/map',
                                 lambda m: self._set('leo2', m), qos)
        self.create_subscription(OccupancyGrid, '/shared_map',
                                 lambda m: self._set('shared', m), qos)
        self.create_subscription(
            TransformStamped, '/vetted_transform/leo2_to_leo1',
            self._on_tf, 10)
        self.create_subscription(Bool, '/alignment_locked',
                                 lambda m: setattr(self, 'locked', bool(m.data)), 10)
        self.create_subscription(Float32, '/tag_alignment_confidence',
                                 lambda m: self.conf.__setitem__('tag', float(m.data)), 10)
        self.create_subscription(Float32, '/alignment_confidence',
                                 lambda m: self.conf.__setitem__('map', float(m.data)), 10)
        for ns in ('leo1', 'leo2'):
            self.create_subscription(
                String, f'/{ns}/frontier_explorer/status',
                lambda m, n=ns: self._on_status(n, m), 10)
            self.create_subscription(
                MarkerArray, f'/{ns}/frontier_explorer/frontiers',
                lambda m, n=ns: self._on_frontiers(n, m), 10)
            self.create_subscription(
                MarkerArray, f'/{ns}/tag_detections',
                lambda m, n=ns: self._on_tags(n, m), 10)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(interval, self._snap)
        self.get_logger().info(f'merge_timelapse -> {out_dir} every {interval}s')

    def _set(self, key, msg):
        self.maps[key] = msg

    def _on_tf(self, msg):
        t = msg.transform
        self.tf_est = (float(t.translation.x), float(t.translation.y),
                       2.0 * math.atan2(t.rotation.z, t.rotation.w))

    def _on_status(self, ns, msg):
        try:
            self.status[ns] = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def _on_frontiers(self, ns, msg):
        pts = []
        for mk in msg.markers:
            if mk.action == 2:        # DELETE
                continue
            pts.append((float(mk.pose.position.x), float(mk.pose.position.y)))
        self.frontiers[ns] = pts[:60]

    def _on_tags(self, ns, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        for mk in msg.markers:
            mid = int(mk.id)
            if mid not in self.tags_seen[ns]:
                # First time this rover ever confirmed this tag -- the moment
                # that matters for the rendezvous story.
                self.tag_events.append((round(now, 1), ns, mid))
                self.get_logger().info(f'{ns} first saw tag {mid} at t={now:.0f}s')
            self.tags_seen[ns][mid] = (float(mk.pose.position.x),
                                       float(mk.pose.position.y))

    def _pose(self, ns):
        try:
            tf = self.buf.lookup_transform(f'{ns}/map', f'{ns}/base_link',
                                           rclpy.time.Time(),
                                           timeout=Duration(seconds=0.05))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return (float('nan'), float('nan'))

    def _snap(self):
        if self.maps['leo1'] is None and self.maps['leo2'] is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        payload = {'t': np.float64(t),
                   'locked': np.bool_(self.locked),
                   'p1': np.array(self._pose('leo1'), dtype=np.float64),
                   'p2': np.array(self._pose('leo2'), dtype=np.float64),
                   'tf': np.array(self.tf_est if self.tf_est else
                                  (np.nan, np.nan, np.nan), dtype=np.float64)}
        for ns in ('leo1', 'leo2'):
            st = self.status.get(ns) or {}
            g = st.get('goal')
            payload[f'{ns}_goal'] = np.array(g if g else (np.nan, np.nan),
                                             dtype=np.float64)
            payload[f'{ns}_state'] = np.str_(str(st.get('state', '')))
            payload[f'{ns}_kind'] = np.str_(str(st.get('goal_kind', '')))
            payload[f'{ns}_nfront'] = np.int32(int(st.get('frontiers') or 0))
            fr = self.frontiers.get(ns) or []
            payload[f'{ns}_frontiers'] = (np.array(fr, dtype=np.float64)
                                          if fr else np.zeros((0, 2)))
            tg = self.tags_seen.get(ns) or {}
            payload[f'{ns}_tagids'] = np.array(sorted(tg), dtype=np.int32)
            payload[f'{ns}_tagpos'] = (np.array([tg[k] for k in sorted(tg)],
                                                dtype=np.float64)
                                       if tg else np.zeros((0, 2)))
        payload['conf'] = np.array([self.conf['tag'], self.conf['map']],
                                   dtype=np.float64)
        common = sorted(set(self.tags_seen['leo1']) & set(self.tags_seen['leo2']))
        payload['common'] = np.array(common, dtype=np.int32)
        for key in ('leo1', 'leo2', 'shared'):
            g, info = _grid(self.maps[key])
            payload[key] = g if g is not None else np.zeros((1, 1), dtype=np.int8)
            payload[f'{key}_info'] = np.array(info, dtype=np.float64)
        path = os.path.join(self.out_dir, f'snap{self.n:04d}.npz')
        np.savez_compressed(path, **payload)
        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(f'{self.n} snapshots, t={t:.0f}s '
                                   f'locked={int(self.locked)}')


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/timelapse'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    node = MergeTimelapse(out_dir, interval)
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
