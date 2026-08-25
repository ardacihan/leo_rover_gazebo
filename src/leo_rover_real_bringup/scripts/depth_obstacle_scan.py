#!/usr/bin/env python3

"""Project a depth image to 3D, keep only points at obstacle height, and emit
a LaserScan of what is genuinely in the way.

Why not depthimage_to_laserscan: it collapses image rows without knowing where
they land in the world, so a forward-facing camera reports the floor as a wall.
Measured on Rover 4, a 40-row band saw the floor at 1.44 m and pinned the scan
there regardless of the real scene.

This node instead reconstructs each pixel's 3D position, transforms it into the
base frame, and discards anything below `min_obstacle_height` (the floor) or
above `max_obstacle_height` (door frames, ceilings). What survives is binned by
bearing, keeping the nearest return per bin.

That also makes the camera worth having: Rover 4's LIDAR sits at 0.246 m and
the camera at 0.250 m, so a fixed-row band would only duplicate the LIDAR
plane. Filtering by height is what lets it see the low boxes and table tops
the LIDAR misses.

Publishes only a LaserScan. It never publishes velocity commands.
"""

import math

import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import tf2_ros


class DepthObstacleScan(Node):
    def __init__(self):
        super().__init__("depth_obstacle_scan")

        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/camera_scan")
        self.declare_parameter("target_frame", "base_footprint")
        # Floor rejection.  0.06 m clears floor noise without hiding a low box.
        self.declare_parameter("min_obstacle_height", 0.06)
        # Above the rover's own height nothing can obstruct it.
        self.declare_parameter("max_obstacle_height", 0.60)
        self.declare_parameter("range_min", 0.45)
        self.declare_parameter("range_max", 6.0)
        self.declare_parameter("angle_increment_degrees", 1.0)
        # Every Nth pixel in each axis.  2 -> a quarter of the pixels, which
        # keeps this well clear of the CPU budget that Collision Monitor needs.
        self.declare_parameter("pixel_step", 2)
        self.declare_parameter("min_points_per_bin", 2)
        # Every link from base_footprint to the depth frame is static: the
        # URDF's base_footprint->base_link, our base_link->camera_link, and
        # the RealSense's internal frames.  Measured on Rover 4, rclpy's
        # TransformListener costs ~26% of a core deserialising /tf at 120 Hz,
        # twice this node's real work, so resolve once and release it.
        # Set false if the camera is ever put on a moving mount.
        self.declare_parameter("assume_static_transform", True)

        self.target_frame = self.get_parameter("target_frame").value
        self.min_height = float(self.get_parameter("min_obstacle_height").value)
        self.max_height = float(self.get_parameter("max_obstacle_height").value)
        if self.max_height <= self.min_height:
            raise RuntimeError("max_obstacle_height must exceed min_obstacle_height")
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.step = max(int(self.get_parameter("pixel_step").value), 1)
        self.min_points = max(int(self.get_parameter("min_points_per_bin").value), 1)

        self.angle_increment = math.radians(
            float(self.get_parameter("angle_increment_degrees").value)
        )
        self.angle_min = -math.pi
        self.bin_count = int(round(2.0 * math.pi / self.angle_increment))

        self.static_transform = bool(
            self.get_parameter("assume_static_transform").value
        )
        self.info = None
        self._rays = None
        self._ray_shape = None
        self._cached_transform = None
        self._listener_released = False

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value,
            self._info_callback, 10,
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self._depth_callback, qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"depth obstacle scan -> {self.target_frame}; keeping "
            f"{self.min_height:.2f}-{self.max_height:.2f} m above ground, "
            f"{self.range_min:.2f}-{self.range_max:.2f} m, "
            f"every {self.step} px"
        )
        self._warned = False

    def _info_callback(self, msg):
        self.info = msg

    def _unit_rays(self, height, width):
        """Per-pixel camera-optical-frame direction vectors. Cached."""
        if self._ray_shape == (height, width):
            return self._rays

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        rows = numpy.arange(0, height, self.step, dtype=numpy.float32)
        cols = numpy.arange(0, width, self.step, dtype=numpy.float32)
        # Optical frame: x right, y down, z forward.
        x = (cols[None, :] - cx) / fx
        y = (rows[:, None] - cy) / fy
        z = numpy.ones((rows.size, cols.size), dtype=numpy.float32)
        self._rays = (x * z, numpy.broadcast_to(y, z.shape) * z, z)
        self._ray_shape = (height, width)
        return self._rays

    @staticmethod
    def _matrix(transform):
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = numpy.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=numpy.float32)
        t = transform.transform.translation
        return rotation, numpy.array([t.x, t.y, t.z], dtype=numpy.float32)

    def _release_listener(self):
        """Stop paying for /tf once a static transform has been resolved."""
        if self._listener_released:
            return
        for attribute in ("tf_sub", "tf_static_sub"):
            subscription = getattr(self.listener, attribute, None)
            if subscription is None:
                continue
            try:
                self.destroy_subscription(subscription)
            except Exception:  # noqa: BLE001 - best effort; correctness is unaffected
                pass
        self._listener_released = True
        self.get_logger().info(
            "camera transform resolved and cached; released the TF listener"
        )

    def _lookup(self, source_frame):
        if self._cached_transform is not None:
            return self._cached_transform
        try:
            transform = self.buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time()
            )
        except Exception as error:  # noqa: BLE001 - fail closed, keep running
            if not self._warned:
                self.get_logger().warn(
                    f"no transform {source_frame} -> {self.target_frame}: "
                    f"{error}"
                )
                self._warned = True
            return None

        resolved = self._matrix(transform)
        if self.static_transform:
            self._cached_transform = resolved
            self._release_listener()
        return resolved

    def _depth_callback(self, msg):
        if self.info is None:
            return
        resolved = self._lookup(msg.header.frame_id)
        if resolved is None:
            return

        # Decimate before converting so only the kept pixels are widened.
        if msg.encoding == "16UC1":
            raw = numpy.frombuffer(msg.data, dtype=numpy.uint16)
            raw = raw.reshape(msg.height, msg.width)[::self.step, ::self.step]
            depth = raw.astype(numpy.float32)
            depth *= 1e-3
        elif msg.encoding == "32FC1":
            raw = numpy.frombuffer(msg.data, dtype=numpy.float32)
            depth = raw.reshape(msg.height, msg.width)[::self.step, ::self.step]
        else:
            self.get_logger().warn(f"unsupported depth encoding {msg.encoding}")
            return

        ux, uy, uz = self._unit_rays(msg.height, msg.width)

        valid = numpy.isfinite(depth) & (depth > 0.0) & (depth <= self.range_max)
        if not valid.any():
            self._publish(msg.header.stamp, None)
            return

        d = depth[valid]
        points = numpy.stack((ux[valid] * d, uy[valid] * d, uz[valid] * d))

        rotation, translation = resolved
        base = rotation @ points + translation[:, None]

        # The height filter is the whole point: it is what separates the floor
        # from an obstacle, which a row-based conversion cannot do.
        keep = (base[2] >= self.min_height) & (base[2] <= self.max_height)
        if not keep.any():
            self._publish(msg.header.stamp, None)
            return

        bx, by = base[0][keep], base[1][keep]
        planar = numpy.hypot(bx, by)
        near = (planar >= self.range_min) & (planar <= self.range_max)
        if not near.any():
            self._publish(msg.header.stamp, None)
            return

        bearing = numpy.arctan2(by[near], bx[near])
        planar = planar[near]

        index = numpy.clip(
            ((bearing - self.angle_min) / self.angle_increment).astype(int),
            0, self.bin_count - 1,
        )
        ranges = numpy.full(self.bin_count, numpy.inf, dtype=numpy.float32)

        # numpy.minimum.at would be the obvious way to reduce per bin, but it
        # is an unbuffered ufunc and measured ~3x the cost of this node's
        # entire remaining work.  Sorting by range and taking each bin's first
        # occurrence gives the same per-bin minimum from vectorised calls.
        order = numpy.argsort(planar, kind="stable")
        bins_sorted = index[order]
        unique_bins, first = numpy.unique(bins_sorted, return_index=True)
        ranges[unique_bins] = planar[order][first]

        # A single stray pixel is noise, not an obstacle.
        if self.min_points > 1:
            counts = numpy.bincount(index, minlength=self.bin_count)
            ranges[counts < self.min_points] = numpy.inf

        self._publish(msg.header.stamp, ranges)

    def _publish(self, stamp, ranges):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_min + self.angle_increment * (self.bin_count - 1))
        scan.angle_increment = float(self.angle_increment)
        scan.time_increment = 0.0
        scan.scan_time = 0.067
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        if ranges is None:
            ranges = numpy.full(self.bin_count, numpy.inf, dtype=numpy.float32)
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthObstacleScan()
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
