"""Resample laser scans onto a fixed angular grid for slam_toolbox.

The RPLidar driver emits raw revolutions: every scan has a different number
of rays (505-513 observed), a different angle_min and a different
angle_increment. Karto templates its laser model on the FIRST scan it sees
and silently rejects every scan whose ray count differs ("LaserRangeScan
contains N range readings, expected M"), which on 2026-08-21 threw away most
scans and left the SLAM map thin and speckled.

This node rebins each scan onto a constant grid (num_bins rays spanning
[-pi, pi)), taking the minimum range per bin, so every scan slam_toolbox
receives is identical in shape and always accepted.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):

    def __init__(self):
        super().__init__('scan_normalizer')
        self.declare_parameter('input_topic', '/scan_filtered')
        self.declare_parameter('output_topic', '/scan_uniform')
        self.declare_parameter('num_bins', 512)
        g = lambda n: self.get_parameter(n).value
        self.n = int(g('num_bins'))
        self.amin = -np.pi
        self.ainc = 2.0 * np.pi / self.n
        self.pub = self.create_publisher(LaserScan, str(g('output_topic')), 5)
        self.create_subscription(LaserScan, str(g('input_topic')),
                                 self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"normalizing {g('input_topic')} -> {g('output_topic')} "
            f"to {self.n} rays at {np.degrees(self.ainc):.3f} deg")

    def on_scan(self, msg):
        src = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(src)) * msg.angle_increment
        valid = np.isfinite(src) & (src >= msg.range_min) & (src <= msg.range_max)
        out = np.full(self.n, np.inf, dtype=np.float32)
        if valid.any():
            idx = np.round((angles[valid] - self.amin) / self.ainc).astype(int) % self.n
            np.minimum.at(out, idx, src[valid])
        res = LaserScan()
        res.header = msg.header
        res.angle_min = float(self.amin)
        res.angle_max = float(self.amin + (self.n - 1) * self.ainc)
        res.angle_increment = float(self.ainc)
        res.time_increment = float(msg.scan_time / self.n) if msg.scan_time else 0.0
        res.scan_time = msg.scan_time
        res.range_min = msg.range_min
        res.range_max = msg.range_max
        res.ranges = out.tolist()
        self.pub.publish(res)


def main():
    rclpy.init()
    node = ScanNormalizer()
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
