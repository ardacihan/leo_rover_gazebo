#!/usr/bin/env python3
"""Throttle the RealSense colour stream for debug bags.

Two outputs, and which one you record decides whether the bag is 20 MB/min or
280 MB/min:

    /debug/color_5hz/compressed   CompressedImage, the driver's OWN jpeg,
                                  passed through untouched. ~150 kB/frame.
                                  RECORD THIS.
    /debug/color_5hz              sensor_msgs/Image, raw rgb8. 921 kB/frame at
                                  640x480 -- 4.6 MB/s at 5 Hz. Only published
                                  when RAW=1, and only worth it for a tool that
                                  cannot decode jpeg.

Measured on the 2026-08-20 drive bag: colour + depth were 92% of 723 MB over
9.6 min; everything else (lidar, odom, TF, IMU, cmd chain) came to ~6 MB/min.
The camera is the only thing worth throttling, and the compressed topic is
where the win is -- the raw republish was never a size fix, only a CPU one
(bagging the raw topic at 15 Hz pushed load to 9.3/6 cores and opened
0.4-0.5 s scan gaps that tripped the explorer watchdogs).

Env:
    HZ           output rate cap, default 5.0
    RAW          set to 1 to also publish the raw Image topic
    COLOR_TOPIC  raw source, default /camera/camera/color/image_raw
                 (the compressed source is that + '/compressed')
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class ColorThrottle(Node):
    def __init__(self):
        super().__init__('debug_color_throttle')
        self.period = 1.0 / float(os.environ.get('HZ', '5.0'))
        self.raw = os.environ.get('RAW', '0') == '1'
        color = os.environ.get('COLOR_TOPIC',
                               '/camera/camera/color/image_raw')
        info = color.rsplit('/', 1)[0] + '/camera_info'

        self.info = None
        self.last = {'raw': 0.0, 'jpg': 0.0}
        self.n = {'raw': 0, 'jpg': 0}

        self.pub_jpg = self.create_publisher(
            CompressedImage, '/debug/color_5hz/compressed', 5)
        self.pub_info = self.create_publisher(
            CameraInfo, '/debug/color_5hz/camera_info', 5)
        self.create_subscription(CompressedImage, color + '/compressed',
                                 self._on_jpg, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info, self._on_info,
                                 qos_profile_sensor_data)

        self.pub_raw = None
        if self.raw:
            self.pub_raw = self.create_publisher(Image, '/debug/color_5hz', 5)
            self.create_subscription(Image, color, self._on_raw,
                                     qos_profile_sensor_data)

        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'{color}/compressed -> /debug/color_5hz/compressed at '
            f'{1.0 / self.period:.1f} Hz' + ('; raw ALSO on' if self.raw else ''))

    def _due(self, kind):
        t = time.monotonic()
        if t - self.last[kind] < self.period:
            return False
        self.last[kind] = t
        return True

    def _on_info(self, msg):
        self.info = msg

    def _on_jpg(self, msg):
        if not self._due('jpg'):
            return
        self.pub_jpg.publish(msg)
        if self.info is not None:
            self.pub_info.publish(self.info)
        self.n['jpg'] += 1

    def _on_raw(self, msg):
        if not self._due('raw'):
            return
        self.pub_raw.publish(msg)
        self.n['raw'] += 1

    def _report(self):
        if self.n['jpg'] == 0 and self.n['raw'] == 0:
            # The compressed topic only exists if image_transport plugins are
            # installed alongside the RealSense driver. Silence here means the
            # bag will have no camera at all, which is worth saying out loud.
            self.get_logger().warn(
                'no colour frames yet -- check that '
                f'{os.environ.get("COLOR_TOPIC", "/camera/camera/color/image_raw")}'
                '/compressed exists (ros2 topic list | grep compressed)')
        else:
            self.get_logger().info(
                f'forwarded {self.n["jpg"]} jpeg, {self.n["raw"]} raw frames')


def main():
    rclpy.init()
    node = ColorThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
