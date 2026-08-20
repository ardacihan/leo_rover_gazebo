#!/usr/bin/env python3
"""Feed the costmap's camera layer from the bag's compressed depth stream.

The real rover publishes /camera/camera/depth/color/points (PointCloud2) from
the RealSense driver; the drive bag only carries /bag/depth/compressed (16UC1
PNG, mm) plus its CameraInfo. This node rebuilds a decimated cloud in the
depth optical frame so the ObstacleLayer sees exactly what the camera saw.

    python3 depth_to_points.py --ros-args -p use_sim_time:=true
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField

try:
    import cv2
except ImportError:
    cv2 = None

DECIM = 4          # every 4th pixel in x and y
MIN_M, MAX_M = 0.2, 3.2


class DepthToPoints(Node):

    def __init__(self):
        super().__init__('depth_to_points')
        self.info = None
        self.last_pub = None
        qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, '/rob_4/camera/depth/camera_info',
                                 self.on_info, qos)
        self.create_subscription(CompressedImage, '/bag/depth/compressed',
                                 self.on_depth, qos)
        self.pub = self.create_publisher(
            PointCloud2, '/camera/camera/depth/color/points', 2)
        self.get_logger().info('depth_to_points up')

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        if self.info is None or cv2 is None:
            return
        t = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_pub is not None and (t - self.last_pub).nanoseconds < 2e8:
            return  # ~5 Hz is what the costmap consumes anyway
        d = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if d is None:
            return
        self.last_pub = t
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        z = d[::DECIM, ::DECIM].astype(np.float32) * 1e-3
        h, w = z.shape
        us = (np.arange(w, dtype=np.float32) * DECIM - cx) / fx
        vs = (np.arange(h, dtype=np.float32) * DECIM - cy) / fy
        uu, vv = np.meshgrid(us, vs)
        ok = (z > MIN_M) & (z < MAX_M)
        pts = np.stack([(uu * z)[ok], (vv * z)[ok], z[ok]], axis=1)

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'camera_depth_optical_frame'
        out.height = 1
        out.width = pts.shape[0]
        out.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyz')
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * pts.shape[0]
        out.is_dense = True
        out.data = pts.astype(np.float32).tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthToPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
