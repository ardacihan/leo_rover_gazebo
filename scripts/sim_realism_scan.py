#!/usr/bin/env python3
"""Degrade the sim's ideal lidar into something like the rover's RPLIDAR C1.

The Gazebo ``gpu_lidar`` in ``leo_rover_with_sensors.urdf.xacro`` has no
``<noise>`` block and a 20 m range, so every scan is geometrically exact out to
the far wall. The physical unit is a 10 Hz, ~12 m, centimetre-noise device, and
REAL_ROVER_GUIDE.md records a fixed self-return from the camera bracket between
+45 deg and +82 deg at 0.06-0.17 m.

This node republishes ``/leo1/scan`` with:

  * additive Gaussian range noise,
  * a hard max-range clamp (returns beyond it become +inf, as on real hardware),
  * a minimum range below which returns are dropped,
  * random per-beam dropouts,
  * an optional camera-bracket self-return sector,
  * an optional frame rename, so a deliberately *miscalibrated* static transform
    can be inserted between the true mount and the frame SLAM believes in.

Usage (inside the sim container):

    python3 /ros2_ws/scripts/sim_realism_scan.py --ros-args \
        -p range_noise:=0.02 -p range_max:=12.0 -p self_return:=true
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def degrade_ranges(ranges, angle_min, angle_increment, rng,
                   range_noise=0.02, range_max=12.0, range_min=0.05,
                   dropout_rate=0.02, self_return=True,
                   self_lo=math.radians(45.0), self_hi=math.radians(82.0),
                   self_r_lo=0.06, self_r_hi=0.17):
    """Return (raw_like, self_filtered) range lists for one scan.

    Pure function so the degradation can be replayed and plotted offline with
    exactly the code the sim runs.
    """
    out, masked_out = [], []
    angle = angle_min
    for r in ranges:
        value = float('inf')
        if r is not None and math.isfinite(r):
            noisy = r + rng.gauss(0.0, range_noise)
            if range_min <= noisy <= range_max:
                value = noisy
        if dropout_rate > 0.0 and rng.random() < dropout_rate:
            value = float('inf')
        masked = value
        if self_return and self_lo <= angle <= self_hi:
            # The bracket occludes the beam: it always returns short.
            value = rng.uniform(self_r_lo, self_r_hi)
            # The self-mask is bounded by distance, so a real obstacle further
            # out in the same direction still survives.
            masked = float('inf')
        out.append(value)
        masked_out.append(masked)
        angle += angle_increment
    return out, masked_out


class RealisticScan(Node):

    def __init__(self):
        super().__init__('sim_realism_scan')
        self.declare_parameter('input_topic', '/leo1/scan')
        self.declare_parameter('output_topic', '/leo1/scan_real')
        self.declare_parameter('output_frame', '')     # '' = keep incoming frame
        self.declare_parameter('range_noise', 0.02)    # m, 1-sigma
        self.declare_parameter('range_max', 12.0)      # m, RPLIDAR C1
        # RPLIDAR C1 spec minimum. Reported in the message, so karto keeps every
        # return above it -- including the camera-bracket self-return, exactly as
        # the physical stack does when it feeds raw /scan to slam_toolbox.
        self.declare_parameter('range_min', 0.05)      # m
        self.declare_parameter('dropout_rate', 0.02)   # fraction of beams
        self.declare_parameter('self_return', True)
        self.declare_parameter('self_angle_min_deg', 45.0)
        self.declare_parameter('self_angle_max_deg', 82.0)
        self.declare_parameter('self_range_min', 0.06)
        self.declare_parameter('self_range_max', 0.17)
        self.declare_parameter('seed', 0)

        self.output_frame = self.get_parameter('output_frame').value
        self.range_noise = float(self.get_parameter('range_noise').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.dropout_rate = float(self.get_parameter('dropout_rate').value)
        self.self_return = bool(self.get_parameter('self_return').value)
        self.self_lo = math.radians(
            float(self.get_parameter('self_angle_min_deg').value))
        self.self_hi = math.radians(
            float(self.get_parameter('self_angle_max_deg').value))
        self.self_r_lo = float(self.get_parameter('self_range_min').value)
        self.self_r_hi = float(self.get_parameter('self_range_max').value)
        self.rng = random.Random(int(self.get_parameter('seed').value))

        base = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(
            LaserScan, base, qos_profile_sensor_data)
        # Second output with the bracket sector masked, mirroring the physical
        # stack's /scan_self_filtered. Publishing both from one node means the
        # two topics share a noise realisation, so an A/B run differs only in
        # the self-return.
        self.pub_filtered = self.create_publisher(
            LaserScan, base + '_selffiltered', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value,
            self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'degrading scan: noise={self.range_noise} m, '
            f'max={self.range_max} m, dropouts={self.dropout_rate}, '
            f'self_return={self.self_return}; publishing {base} and '
            f'{base}_selffiltered')

    def _shell(self, msg):
        out = LaserScan()
        out.header = msg.header
        if self.output_frame:
            out.header.frame_id = self.output_frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.range_min
        out.range_max = self.range_max
        return out

    def on_scan(self, msg):
        ranges, filtered = degrade_ranges(
            msg.ranges, msg.angle_min, msg.angle_increment, self.rng,
            range_noise=self.range_noise, range_max=self.range_max,
            range_min=self.range_min, dropout_rate=self.dropout_rate,
            self_return=self.self_return, self_lo=self.self_lo,
            self_hi=self.self_hi, self_r_lo=self.self_r_lo,
            self_r_hi=self.self_r_hi)

        raw = self._shell(msg)
        raw.ranges = ranges
        self.pub.publish(raw)

        clean = self._shell(msg)
        clean.ranges = filtered
        self.pub_filtered.publish(clean)


def main():
    rclpy.init()
    node = RealisticScan()
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
