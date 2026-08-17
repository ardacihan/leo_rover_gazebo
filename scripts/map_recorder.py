#!/usr/bin/env python3
"""Record the SLAM map + robot trail as PNG snapshots and a time-lapse video.

Usage (inside the container):
    python3 /ros2_ws/scripts/map_recorder.py <output_basename> [interval_sec]

Writes <output_basename>.mp4 (or .avi fallback) and <output_basename>_final.png
on SIGINT/SIGTERM, plus incremental saves every 50 frames so artifacts survive
a hard kill.
"""

import signal
import sys

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

UNKNOWN_GRAY = 128


class MapRecorder(Node):
    def __init__(self, out_base, interval):
        super().__init__('map_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.out_base = out_base
        self.frames = []          # (img, origin_x, origin_y, res)
        self.trail = []           # world (x, y)
        self.msg = None
        self.base_frame = 'leo1/base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        self.create_timer(interval, self._snap)

    def _cb(self, msg):
        self.msg = msg

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def _snap(self):
        if self.msg is None:
            return
        pos = self._robot_xy()
        if pos is not None:
            self.trail.append(pos)
        info = self.msg.info
        grid = np.asarray(self.msg.data, dtype=np.int16).reshape(
            info.height, info.width)
        img = np.full(grid.shape, UNKNOWN_GRAY, dtype=np.uint8)
        img[(grid >= 0) & (grid < 50)] = 255
        img[grid >= 50] = 0
        self.frames.append((img, info.origin.position.x,
                            info.origin.position.y, info.resolution,
                            len(self.trail)))
        if len(self.frames) % 50 == 0:
            self.save()
            print(f'autosaved at {len(self.frames)} frames', flush=True)

    def _render(self, frame, x0, y0, res, x1, y1, w, h, trail_n):
        img, ox, oy, fres, _ = frame
        canvas = np.full((h, w), UNKNOWN_GRAY, dtype=np.uint8)
        cx = int(round((ox - x0) / res))
        cy = int(round((oy - y0) / res))
        canvas[cy:cy + img.shape[0], cx:cx + img.shape[1]] = img
        bgr = cv2.cvtColor(np.flipud(canvas), cv2.COLOR_GRAY2BGR)
        for i, (tx, ty) in enumerate(self.trail[:trail_n]):
            px = int((tx - x0) / res)
            py = h - 1 - int((ty - y0) / res)
            if 0 <= px < w and 0 <= py < h:
                bgr[py, px] = (0, 0, 255)
        if trail_n and self.trail:
            tx, ty = self.trail[min(trail_n, len(self.trail)) - 1]
            px = int((tx - x0) / res)
            py = h - 1 - int((ty - y0) / res)
            cv2.circle(bgr, (px, py), 4, (255, 0, 0), -1)
        return bgr

    def save(self):
        if not self.frames:
            print('no frames captured', flush=True)
            return
        res = self.frames[-1][3]
        x0 = min(f[1] for f in self.frames)
        y0 = min(f[2] for f in self.frames)
        x1 = max(f[1] + f[0].shape[1] * f[3] for f in self.frames)
        y1 = max(f[2] + f[0].shape[0] * f[3] for f in self.frames)
        w = int(round((x1 - x0) / res)) + 2
        h = int(round((y1 - y0) / res)) + 2

        last = self._render(self.frames[-1], x0, y0, res, x1, y1, w, h,
                            len(self.trail))
        cv2.imwrite(self.out_base + '_final.png', last)

        # Exploration path as a simple CSV of map-frame poses (x, y).
        path_csv = self.out_base + '_path.csv'
        with open(path_csv, 'w', encoding='utf-8') as f:
            f.write('x,y\n')
            for tx, ty in self.trail:
                f.write(f'{tx:.4f},{ty:.4f}\n')
        print(f'path written: {path_csv} ({len(self.trail)} poses)', flush=True)

        # mp4v first; MJPG/avi fallback for restricted opencv builds
        # 'avc1' (H.264) first: 'mp4v' is MPEG-4 Part 2, which plays in VLC but
        # NO browser will decode it, so those files look silently broken on any
        # web page. Fall back only if this OpenCV build lacks an H.264 encoder.
        for ext, fourcc in (('.mp4', 'avc1'), ('.mp4', 'mp4v'), ('.avi', 'MJPG')):
            vw = cv2.VideoWriter(self.out_base + ext,
                                 cv2.VideoWriter_fourcc(*fourcc), 5,
                                 (w, h))
            if not vw.isOpened():
                continue
            for f in self.frames:
                vw.write(self._render(f, x0, y0, res, x1, y1, w, h, f[4]))
            vw.release()
            print(f'video written: {self.out_base}{ext} '
                  f'({len(self.frames)} frames, {w}x{h})', flush=True)
            return
        print('no usable video codec found', flush=True)


def main():
    out_base = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/reports/run'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    node = MapRecorder(out_base, interval)

    def _stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
