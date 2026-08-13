#!/usr/bin/env python3

"""Project aligned depth into base-frame, ground-filtered collision and SLAM scans."""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import tf2_ros

from sensor_geometry import (
    points_to_scan_ranges,
    project_depth_to_base,
    quaternion_rotation_matrix,
)


class DepthHeightFilter(Node):
    """Keep obstacle-height depth points after transforming them into the rover base."""

    def __init__(self):
        super().__init__("depth_height_filter")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info")
        self.declare_parameter("collision_scan_topic", "/camera/scan_collision")
        self.declare_parameter("slam_scan_topic", "/camera/scan_slam")
        self.declare_parameter("base_frame", "base_footprint")
        # Stride 2 keeps a quarter of a 640x480 frame, about 77k points, and
        # reprojecting that at ~14 Hz in numpy measured 343% CPU on the Jetson's
        # six cores. The resulting load average of 7.9 starved the DDS receive
        # threads: firmware telemetry kept arriving but went unprocessed, which
        # looked exactly like the rover's firmware dying. Stride 4 costs a
        # quarter of the work and still yields far more points than the scan has
        # bins, so no angular resolution is lost.
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("valid_depth_min", 0.10)
        self.declare_parameter("valid_depth_max", 10.0)
        self.declare_parameter("minimum_valid_depth_fraction", 0.05)
        self.declare_parameter("collision_min_height", 0.04)
        self.declare_parameter("collision_max_height", 0.45)
        self.declare_parameter("slam_min_height", 0.18)
        self.declare_parameter("slam_max_height", 0.31)
        self.declare_parameter("angle_min", -1.05)
        self.declare_parameter("angle_max", 1.05)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("scan_time", 1.0 / 15.0)
        self.declare_parameter("range_min", 0.20)
        self.declare_parameter("collision_range_max", 3.0)
        self.declare_parameter("slam_range_max", 5.0)
        self.declare_parameter("transform_timeout", 0.10)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.pixel_stride = max(int(self.get_parameter("pixel_stride").value), 1)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.valid_depth_min = float(self.get_parameter("valid_depth_min").value)
        self.valid_depth_max = float(self.get_parameter("valid_depth_max").value)
        self.minimum_valid_fraction = float(
            self.get_parameter("minimum_valid_depth_fraction").value
        )
        self.collision_min_height = float(self.get_parameter("collision_min_height").value)
        self.collision_max_height = float(self.get_parameter("collision_max_height").value)
        self.slam_min_height = float(self.get_parameter("slam_min_height").value)
        self.slam_max_height = float(self.get_parameter("slam_max_height").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.scan_time = float(self.get_parameter("scan_time").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.collision_range_max = float(self.get_parameter("collision_range_max").value)
        self.slam_range_max = float(self.get_parameter("slam_range_max").value)
        self.transform_timeout = float(self.get_parameter("transform_timeout").value)

        if not 0.0 <= self.minimum_valid_fraction <= 1.0:
            raise ValueError("minimum_valid_depth_fraction must be in [0, 1]")
        if not 0.0 <= self.collision_min_height < self.collision_max_height:
            raise ValueError("invalid collision height band")
        if not 0.0 <= self.slam_min_height < self.slam_max_height:
            raise ValueError("invalid SLAM height band")

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.last_log_time = 0.0
        self.frames_received = 0
        self.frames_published = 0

        self.collision_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("collision_scan_topic").value), qos_profile_sensor_data
        )
        self.slam_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("slam_scan_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "base-frame depth filtering ready: "
            f"collision_z=[{self.collision_min_height:.2f}, {self.collision_max_height:.2f}] m, "
            f"slam_z=[{self.slam_min_height:.2f}, {self.slam_max_height:.2f}] m"
        )

    def _camera_info_callback(self, msg):
        self.camera_info = msg

    def _decode_depth(self, msg):
        encoding = msg.encoding.upper()
        if encoding == "16UC1":
            dtype = ">u2" if msg.is_bigendian else "<u2"
            row_values = int(msg.step) // 2
            image = np.frombuffer(msg.data, dtype=dtype).reshape(int(msg.height), row_values)
            return image[:, : int(msg.width)].astype(np.float32) * self.depth_scale
        if encoding == "32FC1":
            dtype = ">f4" if msg.is_bigendian else "<f4"
            row_values = int(msg.step) // 4
            image = np.frombuffer(msg.data, dtype=dtype).reshape(int(msg.height), row_values)
            return image[:, : int(msg.width)].astype(np.float32)
        raise ValueError(f"unsupported depth encoding {msg.encoding!r}")

    @staticmethod
    def _transform_parts(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        return (
            quaternion_rotation_matrix((q.x, q.y, q.z, q.w)),
            np.asarray((t.x, t.y, t.z), dtype=np.float64),
        )

    def _make_scan(self, msg, ranges, range_max):
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = self.angle_min
        scan.angle_increment = self.angle_increment
        scan.angle_max = self.angle_min + (len(ranges) - 1) * self.angle_increment
        scan.scan_time = self.scan_time
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = range_max
        scan.ranges = ranges.tolist()
        return scan

    def _depth_callback(self, msg):
        self.frames_received += 1
        info = self.camera_info
        if info is None:
            return
        if int(info.width) != int(msg.width) or int(info.height) != int(msg.height):
            self.get_logger().error("depth image and CameraInfo dimensions differ")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.transform_timeout),
            )
            rotation, translation = self._transform_parts(transform)
            depth = self._decode_depth(msg)
            points, valid_fraction = project_depth_to_base(
                depth,
                fx=float(info.k[0]),
                fy=float(info.k[4]),
                cx=float(info.k[2]),
                cy=float(info.k[5]),
                rotation=rotation,
                translation=translation,
                pixel_stride=self.pixel_stride,
                valid_depth_min=self.valid_depth_min,
                valid_depth_max=self.valid_depth_max,
            )
        except (tf2_ros.TransformException, ValueError) as error:
            self.get_logger().warning(f"depth frame rejected: {error}")
            return

        # Do not publish an all-clear scan from a failed/covered depth camera.
        # The command gate then observes a stale camera scan and fails closed.
        if valid_fraction < self.minimum_valid_fraction:
            self.get_logger().warning(
                f"depth frame rejected: valid fraction {valid_fraction:.3f} "
                f"< {self.minimum_valid_fraction:.3f}"
            )
            return

        collision_ranges = points_to_scan_ranges(
            points,
            self.collision_min_height,
            self.collision_max_height,
            self.angle_min,
            self.angle_max,
            self.angle_increment,
            self.range_min,
            self.collision_range_max,
        )
        slam_ranges = points_to_scan_ranges(
            points,
            self.slam_min_height,
            self.slam_max_height,
            self.angle_min,
            self.angle_max,
            self.angle_increment,
            self.range_min,
            self.slam_range_max,
        )
        self.collision_pub.publish(
            self._make_scan(msg, collision_ranges, self.collision_range_max)
        )
        self.slam_pub.publish(self._make_scan(msg, slam_ranges, self.slam_range_max))
        self.frames_published += 1

        now = time.monotonic()
        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f"depth health={valid_fraction:.1%}, base points={len(points)}, "
                f"collision bins={np.count_nonzero(np.isfinite(collision_ranges))}, "
                f"slam bins={np.count_nonzero(np.isfinite(slam_ranges))}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthHeightFilter()
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
