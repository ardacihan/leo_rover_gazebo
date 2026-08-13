#!/usr/bin/env python3

"""Transform LIDAR into the rover base and fuse height-filtered camera scans."""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

from sensor_geometry import (
    merge_planar_points,
    quaternion_rotation_matrix,
    scan_to_base_points,
)


class ScanFusion(Node):
    """Publish base-aligned LIDAR, collision-fused and SLAM-fused scans."""

    def __init__(self):
        super().__init__("scan_fusion")
        self.declare_parameter("lidar_scan_topic", "/scan")
        self.declare_parameter("camera_collision_scan_topic", "/camera/scan_collision")
        self.declare_parameter("camera_slam_scan_topic", "/camera/scan_slam")
        self.declare_parameter("lidar_base_scan_topic", "/scan_lidar_base")
        self.declare_parameter("collision_fused_scan_topic", "/scan_collision_fused")
        self.declare_parameter("slam_fused_scan_topic", "/scan_slam_fused")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_timeout", 0.35)
        self.declare_parameter("transform_timeout", 0.10)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("collision_range_max", 3.0)
        self.declare_parameter("slam_range_max", 12.0)
        self.declare_parameter("self_mask_angle_min_degrees", 12.0)
        self.declare_parameter("self_mask_angle_max_degrees", 83.0)
        self.declare_parameter("self_mask_max_range", 0.22)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_timeout = float(self.get_parameter("camera_timeout").value)
        self.transform_timeout = float(self.get_parameter("transform_timeout").value)
        self.angle_min = -math.pi
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        bins = int(round(2.0 * math.pi / self.angle_increment))
        self.angle_max = self.angle_min + (bins - 1) * self.angle_increment
        self.collision_range_max = float(self.get_parameter("collision_range_max").value)
        self.slam_range_max = float(self.get_parameter("slam_range_max").value)
        self.self_mask_angle_min = math.radians(
            float(self.get_parameter("self_mask_angle_min_degrees").value)
        )
        self.self_mask_angle_max = math.radians(
            float(self.get_parameter("self_mask_angle_max_degrees").value)
        )
        self.self_mask_max_range = float(self.get_parameter("self_mask_max_range").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_collision = None
        self.camera_collision_time = None
        self.camera_slam = None
        self.camera_slam_time = None
        self.last_log_time = 0.0

        self.lidar_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("lidar_base_scan_topic").value),
            qos_profile_sensor_data,
        )
        self.collision_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("collision_fused_scan_topic").value),
            qos_profile_sensor_data,
        )
        self.slam_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("slam_fused_scan_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("camera_collision_scan_topic").value),
            self._camera_collision_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("camera_slam_scan_topic").value),
            self._camera_slam_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("lidar_scan_topic").value),
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "scan fusion ready: all outputs use base-frame directions; "
            f"bounded LIDAR self-mask={math.degrees(self.self_mask_angle_min):.0f}.."
            f"{math.degrees(self.self_mask_angle_max):.0f} deg <= {self.self_mask_max_range:.2f} m"
        )

    def _camera_collision_callback(self, msg):
        self.camera_collision = msg
        self.camera_collision_time = time.monotonic()

    def _camera_slam_callback(self, msg):
        self.camera_slam = msg
        self.camera_slam_time = time.monotonic()

    @staticmethod
    def _transform_parts(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        return (
            quaternion_rotation_matrix((q.x, q.y, q.z, q.w)),
            np.asarray((t.x, t.y, t.z), dtype=np.float64),
        )

    def _scan_points(self, msg, apply_lidar_mask=False):
        if msg.header.frame_id == self.base_frame:
            rotation = np.eye(3)
            translation = np.zeros(3)
        else:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.transform_timeout),
            )
            rotation, translation = self._transform_parts(transform)
        mask_args = {}
        if apply_lidar_mask:
            mask_args = {
                "self_mask_angle_min": self.self_mask_angle_min,
                "self_mask_angle_max": self.self_mask_angle_max,
                "self_mask_max_range": self.self_mask_max_range,
            }
        return scan_to_base_points(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            rotation,
            translation,
            **mask_args,
        )

    def _make_scan(self, source, ranges, range_max):
        output = LaserScan()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self.base_frame
        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment
        output.scan_time = float(source.scan_time)
        output.time_increment = 0.0
        output.range_min = 0.15
        output.range_max = range_max
        output.ranges = ranges.tolist()
        return output

    def _ranges(self, point_sets, range_max):
        return merge_planar_points(
            point_sets,
            self.angle_min,
            self.angle_max,
            self.angle_increment,
            0.15,
            range_max,
        )

    def _lidar_callback(self, msg):
        try:
            lidar_points = self._scan_points(msg, apply_lidar_mask=True)
        except tf2_ros.TransformException as error:
            self.get_logger().warning(f"LIDAR frame rejected: {error}")
            return

        now = time.monotonic()
        collision_points = []
        slam_points = []
        collision_camera_fresh = (
            self.camera_collision is not None
            and self.camera_collision_time is not None
            and now - self.camera_collision_time <= self.camera_timeout
        )
        slam_camera_fresh = (
            self.camera_slam is not None
            and self.camera_slam_time is not None
            and now - self.camera_slam_time <= self.camera_timeout
        )
        try:
            if collision_camera_fresh:
                collision_points = self._scan_points(self.camera_collision)
            if slam_camera_fresh:
                slam_points = self._scan_points(self.camera_slam)
        except tf2_ros.TransformException as error:
            self.get_logger().warning(f"camera scan rejected: {error}")
            collision_camera_fresh = False
            slam_camera_fresh = False
            collision_points = []
            slam_points = []

        lidar_ranges = self._ranges([lidar_points], self.slam_range_max)
        collision_ranges = self._ranges(
            [lidar_points, collision_points], self.collision_range_max
        )
        slam_ranges = self._ranges([lidar_points, slam_points], self.slam_range_max)
        self.lidar_pub.publish(self._make_scan(msg, lidar_ranges, self.slam_range_max))
        self.collision_pub.publish(
            self._make_scan(msg, collision_ranges, self.collision_range_max)
        )
        self.slam_pub.publish(self._make_scan(msg, slam_ranges, self.slam_range_max))

        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f"fused bins: lidar={np.count_nonzero(np.isfinite(lidar_ranges))}, "
                f"collision={np.count_nonzero(np.isfinite(collision_ranges))}, "
                f"slam={np.count_nonzero(np.isfinite(slam_ranges))}; "
                f"camera_collision_fresh={collision_camera_fresh}, "
                f"camera_slam_fresh={slam_camera_fresh}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanFusion()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
