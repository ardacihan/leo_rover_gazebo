#!/usr/bin/env python3
"""Publish the *cheap* versions of the heavy demo topics, for the demo bag.

A presentation bag has to be small enough to hand around, and the two topics
that make a bag big are the camera stream and the SLAM map:

    /leoN/camera/image   640x480 rgb8 @ ~15 Hz  ->  ~13 MB/s raw
    /leoN/map            full OccupancyGrid @ 1 Hz, growing with the map

Neither needs to be recorded at full fidelity for a demo. This node subscribes
to both and republishes:

    /leoN/demo/image/compressed   sensor_msgs/CompressedImage, JPEG,
                                  downscaled to `width` px, at `image_hz`
    /leoN/demo/map                the same OccupancyGrid, at `map_hz`

`scripts/demo_teleop_record.sh` records these two instead of the originals,
which is the whole difference between a ~2 GB bag and a ~50 MB one. The map
copy is republished TRANSIENT_LOCAL so RViz's Map display latches it on
`ros2 bag play` exactly as it does live.

Usage (inside the container):
    python3 demo_bag_feeds.py --ros-args -p robot:=leo1 \
        -p width:=320 -p image_hz:=4.0 -p map_hz:=0.2 -p jpeg_quality:=60
"""

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image


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


class DemoBagFeeds(Node):
    def __init__(self):
        super().__init__('demo_bag_feeds')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        ns = self.declare_parameter('robot', 'leo1').value
        self.width = int(self.declare_parameter('width', 320).value)
        self.quality = int(self.declare_parameter('jpeg_quality', 60).value)
        image_hz = float(self.declare_parameter('image_hz', 4.0).value)
        map_hz = float(self.declare_parameter('map_hz', 0.2).value)
        image_topic = self.declare_parameter(
            'image_topic', f'/{ns}/camera/image').value
        map_topic = self.declare_parameter('map_topic', f'/{ns}/map').value

        self.image_period = 1.0 / image_hz if image_hz > 0 else 0.0
        self.map_period = 1.0 / map_hz if map_hz > 0 else 0.0
        self.last_image = -1e9
        self.last_map = -1e9
        self.n_img = 0
        self.n_map = 0

        # The gz->ros image bridge offers BEST_EFFORT; a RELIABLE subscriber
        # never matches it and this node would sit silent for the whole demo.
        cam_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST)
        # VOLATILE matches slam_toolbox's TRANSIENT_LOCAL /map publisher as
        # well as a plain one; the reverse is not true (see map_coverage.py).
        map_in_qos = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.VOLATILE)
        # Latched out, so a Map display added mid-replay still gets a grid.
        map_out_qos = QoSProfile(depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_img = self.create_publisher(
            CompressedImage, f'/{ns}/demo/image/compressed', 2)
        self.pub_map = self.create_publisher(
            OccupancyGrid, f'/{ns}/demo/map', map_out_qos)
        self.create_subscription(Image, image_topic, self._on_image, cam_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map,
                                 map_in_qos)
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'{image_topic} -> /{ns}/demo/image/compressed '
            f'({self.width}px jpeg q{self.quality} @ {image_hz} Hz); '
            f'{map_topic} -> /{ns}/demo/map @ {map_hz} Hz')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_image(self, msg):
        t = self._now()
        if t - self.last_image < self.image_period:
            return
        try:
            bgr = _to_bgr(msg)
        except (ValueError, IndexError):
            return
        self.last_image = t
        if bgr.shape[1] > self.width:
            h = max(1, int(round(bgr.shape[0] * self.width / bgr.shape[1])))
            bgr = cv2.resize(bgr, (self.width, h), interpolation=cv2.INTER_AREA)
        ok, enc = cv2.imencode(
            '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = enc.tobytes()
        self.pub_img.publish(out)
        self.n_img += 1

    def _on_map(self, msg):
        t = self._now()
        if t - self.last_map < self.map_period:
            return
        self.last_map = t
        self.pub_map.publish(msg)
        self.n_map += 1

    def _report(self):
        self.get_logger().info(
            f'republished {self.n_img} frames, {self.n_map} maps')


def main():
    rclpy.init()
    node = DemoBagFeeds()
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
