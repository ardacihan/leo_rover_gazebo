"""Mock ArUco detector for sim testing until the real detector lands.

Publishes a detection (visualization_msgs/MarkerArray, one Marker per
visible ground-truth marker, pose in the map frame with ~2 cm noise)
whenever the camera is within range, within FOV, looking at the marker's
front face, and has line of sight on the occupancy grid.

The real detector replaces this node behind the same topic; the explorer's
registry only needs (id, pose) pairs.
"""

import math
import random

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .camera_coverage import FREE_MAX, yaw_from_quat


class MockArucoDetector(Node):

    def __init__(self):
        super().__init__('mock_aruco_detector')
        self.declare_parameter('markers_file', '')
        self.declare_parameter('camera_frame', 'leo1/sensor_camera_link')
        self.declare_parameter('map_frame', 'map')
        # Marker ground truth is authored in the common world/'map' frame; in
        # multi-robot own-frame runs (map_frame = leo{i}/map) positions are
        # shifted by the static common->own transform before use.
        self.declare_parameter('common_frame', 'map')
        # LOS occupancy source; per-robot runs pass /leo{i}/map.
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('detection_topic', '/leo1/aruco_detections')
        self.declare_parameter('detection_range', 3.0)
        self.declare_parameter('fov', 1.047)
        self.declare_parameter('max_incidence_deg', 70.0)
        self.declare_parameter('noise_std', 0.02)
        self.declare_parameter('rate', 2.0)

        gp = lambda n: self.get_parameter(n).value
        self.camera_frame = gp('camera_frame')
        self.map_frame = gp('map_frame')
        self.common_frame = gp('common_frame')
        self._markers_shifted = self.map_frame == self.common_frame
        self.range = gp('detection_range')
        self.fov = gp('fov')
        self.cos_max_incidence = math.cos(
            math.radians(gp('max_incidence_deg')))
        self.noise_std = gp('noise_std')

        with open(gp('markers_file')) as f:
            self.markers = yaml.safe_load(f)['markers']
        self.get_logger().info(
            f'Mock detector: {len(self.markers)} ground-truth markers')

        self.map_msg = None
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, gp('map_topic'),
                                 lambda m: setattr(self, 'map_msg', m), qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, gp('detection_topic'), 10)
        self.create_timer(1.0 / gp('rate'), self._tick)

    def _camera_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def _line_of_sight(self, x0, y0, x1, y1):
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        grid = np.asarray(self.map_msg.data, dtype=np.int8).reshape(
            info.height, info.width)
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        d = math.hypot(x1 - x0, y1 - y0)
        steps = max(int(d / (res * 0.7)), 1)
        # stop slightly short of the marker so its own wall doesn't block it
        for i in range(1, int(steps * 0.93)):
            x = x0 + (x1 - x0) * i / steps
            y = y0 + (y1 - y0) * i / steps
            c = int((x - ox) / res)
            r = int((y - oy) / res)
            if r < 0 or c < 0 or r >= info.height or c >= info.width:
                continue
            if grid[r, c] >= FREE_MAX:
                return False
        return True

    def _shift_markers(self):
        """One-time shift of marker positions common frame -> own map frame
        (zero relative yaw between the frames, as everywhere in this stack)."""
        if self._markers_shifted:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.common_frame, rclpy.time.Time())
        except Exception:
            return False
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        for m in self.markers:
            m['x'] += dx
            m['y'] += dy
        self._markers_shifted = True
        self.get_logger().info(
            f'Markers shifted into {self.map_frame} (common->own offset '
            f'({dx:.2f}, {dy:.2f}))')
        return True

    def _tick(self):
        if not self._shift_markers():
            return
        cam = self._camera_pose()
        if cam is None:
            return
        cx, cy, cyaw = cam
        arr = MarkerArray()
        for m in self.markers:
            dx, dy = m['x'] - cx, m['y'] - cy
            dist = math.hypot(dx, dy)
            if dist > self.range or dist < 0.2:
                continue
            bearing = math.atan2(dy, dx)
            if abs(self._norm(bearing - cyaw)) > self.fov / 2.0:
                continue
            nx, ny = math.cos(m['yaw']), math.sin(m['yaw'])
            cos_inc = (-dx * nx - dy * ny) / dist
            if cos_inc < self.cos_max_incidence:
                continue
            if not self._line_of_sight(cx, cy, m['x'], m['y']):
                continue
            det = Marker()
            det.header.frame_id = self.map_frame
            det.header.stamp = self.get_clock().now().to_msg()
            det.ns = 'aruco'
            det.id = m['id']
            det.type = Marker.CUBE
            det.pose.position.x = m['x'] + random.gauss(0, self.noise_std)
            det.pose.position.y = m['y'] + random.gauss(0, self.noise_std)
            det.pose.position.z = m['z']
            det.pose.orientation.z = math.sin(m['yaw'] / 2.0)
            det.pose.orientation.w = math.cos(m['yaw'] / 2.0)
            det.scale.x = det.scale.y = det.scale.z = 0.15
            arr.markers.append(det)
        if arr.markers:
            self.pub.publish(arr)

    @staticmethod
    def _norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = MockArucoDetector()
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
