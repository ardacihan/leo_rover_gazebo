#!/usr/bin/env python3
"""Save camera frames periodically during a run, for the media review.

Grabbing frames at teardown is nearly useless -- whatever the rover happens to
be facing when exploration ends is usually a wall. This runs for the whole run
and keeps a spread of frames, preferring ones where the ArUco detector has
drawn a marker: those are the frames that show whether detection is working at
all, and at what range.

Two kinds of frame are kept, in separate files:
  detNNN_t<sec>.png   a frame from the detector's debug topic (markers drawn)
  rawNNN_t<sec>.png   a plain camera frame, one every `period` seconds

Usage (inside the container):
    python3 frame_grabber.py <outdir> <image_topic> [debug_topic]
                             [period_sec] [max_per_kind]
"""

import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


def _to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray, without depending on cv_bridge."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img.copy()
    if enc in ('mono8', '8uc1'):
        gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(msg.height, msg.step // 4, 4)[:, :msg.width, :]
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    raise ValueError(f'unsupported image encoding {msg.encoding}')


class FrameGrabber(Node):
    def __init__(self, outdir, image_topic, debug_topic, period, max_per_kind):
        super().__init__('frame_grabber')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.period = period
        self.max_per_kind = max_per_kind
        self.counts = {'raw': 0, 'det': 0}
        self.last = {'raw': -1e9, 'det': -1e9}

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, image_topic,
                                 lambda m: self._save('raw', m), qos)
        if debug_topic:
            self.create_subscription(Image, debug_topic,
                                     lambda m: self._save('det', m), qos)
        self.get_logger().info(
            f'frame_grabber -> {outdir} from {image_topic}'
            + (f' + {debug_topic}' if debug_topic else ''))

    def _save(self, kind, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        # Debug frames are worth more, so let them in more often; the detector
        # only publishes one when it actually found a marker worth drawing.
        period = self.period if kind == 'raw' else max(self.period / 4.0, 2.0)
        if t - self.last[kind] < period:
            return
        if self.counts[kind] >= self.max_per_kind:
            return
        try:
            bgr = _to_bgr(msg)
        except (ValueError, IndexError):
            return
        self.last[kind] = t
        n = self.counts[kind]
        self.counts[kind] = n + 1
        path = os.path.join(self.outdir, f'{kind}{n:03d}_t{int(t)}.png')
        cv2.imwrite(path, bgr)


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/frames'
    image_topic = sys.argv[2] if len(sys.argv) > 2 else '/leo1/camera/image'
    debug_topic = sys.argv[3] if len(sys.argv) > 3 else ''
    period = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
    max_per_kind = int(sys.argv[5]) if len(sys.argv) > 5 else 8
    rclpy.init()
    node = FrameGrabber(outdir, image_topic, debug_topic, period, max_per_kind)
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
