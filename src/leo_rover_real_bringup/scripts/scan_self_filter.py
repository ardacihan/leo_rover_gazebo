#!/usr/bin/env python3

"""Mask permanently self-occluded LIDAR sectors before safety consumers see them.

Rover 4 carries a RealSense and mast structure that intrude into the scan plane
behind the rover, producing returns as close as 0.06 m.  Those sit inside
Collision Monitor's 0.31 m footprint circle, so untreated they read as a
permanent obstacle: motion gets clamped, reverse never clears its 0.75 m rear
corridor, and SLAM bakes the robot's own body into the map.

Masked rays are set to NaN rather than to range_max.  NaN means "no reading",
which every downstream consumer already skips; range_max would assert clear
space we cannot actually see.

Only returns closer than `exclusion_max_range` are removed, so a genuine wall
further out in the same direction is still reported.
"""

import math

import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from exploration_policy import parse_sectors


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("scan_yaw_offset", 0.0)
        self.declare_parameter("exclusion_sectors", "")
        self.declare_parameter("exclusion_max_range", 0.45)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        if str(input_topic).rstrip("/") == str(output_topic).rstrip("/"):
            raise RuntimeError("input and output topics must differ")

        raw_offset = float(self.get_parameter("scan_yaw_offset").value)
        self.scan_yaw_offset = math.atan2(
            math.sin(raw_offset), math.cos(raw_offset)
        )
        self.sectors = parse_sectors(
            self.get_parameter("exclusion_sectors").value
        )
        self.exclusion_max_range = float(
            self.get_parameter("exclusion_max_range").value
        )

        self._geometry = None
        self._mask = None
        self._reported = False

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, input_topic, self._callback, qos_profile_sensor_data
        )

        readable = ", ".join(
            f"[{math.degrees(lo):.0f}, {math.degrees(hi):.0f}]"
            for lo, hi in self.sectors
        ) or "none"
        self.get_logger().info(
            f"scan self-filter: {input_topic} -> {output_topic}; "
            f"sectors {readable} closer than "
            f"{self.exclusion_max_range:.2f} m"
        )

    def _sector_mask(self, msg, count):
        """Rays falling inside an excluded sector. Cached per scan geometry."""
        key = (msg.angle_min, msg.angle_increment, count)
        if key == self._geometry:
            return self._mask

        raw = (
            float(msg.angle_min)
            + self.scan_yaw_offset
            + numpy.arange(count, dtype=float) * float(msg.angle_increment)
        )
        angles = numpy.arctan2(numpy.sin(raw), numpy.cos(raw))

        mask = numpy.zeros(count, dtype=bool)
        for low, high in self.sectors:
            mask |= (angles >= low) & (angles <= high)

        self._geometry = key
        self._mask = mask
        return mask

    def _callback(self, msg):
        if not self.sectors:
            self.publisher.publish(msg)
            return

        ranges = numpy.asarray(msg.ranges, dtype=float)
        # The C1 emits a variable ray count per scan, so the mask is rebuilt
        # whenever the geometry changes rather than assumed constant.
        mask = self._sector_mask(msg, ranges.size)
        drop = mask & numpy.isfinite(ranges) & (ranges < self.exclusion_max_range)

        if drop.any():
            ranges = ranges.copy()
            ranges[drop] = math.nan

        if not self._reported:
            self.get_logger().info(
                f"masking {int(numpy.count_nonzero(drop))} of {ranges.size} "
                "rays as self-occlusion"
            )
            self._reported = True

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = ranges.astype(numpy.float32).tolist()
        filtered.intensities = msg.intensities
        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanSelfFilter()
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
