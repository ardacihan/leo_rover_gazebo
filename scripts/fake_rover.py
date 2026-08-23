#!/usr/bin/env python3
"""Minimal stand-in for the rover's sensors, to exercise the command chain.

Publishes a clean 10 Hz /scan, 30 Hz /wheel_odom and the odom -> base_footprint
and base_footprint -> laser transforms -- the three inputs the velocity guard
demands before it will pass a command. Without them the guard correctly zeroes
everything, so a chain test without this looks like a broken chain.
"""
import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class FakeRover(Node):
    def __init__(self):
        super().__init__('fake_rover')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 20)
        self.tfb = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'laser'
        t.transform.translation.z = 0.15
        t.transform.rotation.w = 1.0
        self.static.sendTransform(t)
        # Runtime-settable so the collision monitor can be exercised without
        # restarting anything:  ros2 param set /fake_rover front_range 0.2
        self.declare_parameter('room_range', 3.0)
        self.declare_parameter('front_range', 0.0)   # 0 = no obstacle ahead
        self.declare_parameter('front_width_deg', 30.0)
        self.create_timer(0.1, self.scan)
        self.create_timer(0.033, self.odom)

    def scan(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max = -math.pi, math.pi
        m.angle_increment = 2 * math.pi / 360
        m.range_min, m.range_max = 0.1, 12.0
        room = float(self.get_parameter('room_range').value)
        front = float(self.get_parameter('front_range').value)
        half = int(float(self.get_parameter('front_width_deg').value) / 2)
        ranges = [room] * 360
        if front > 0.0:
            # Beam 180 is straight ahead: angle_min is -pi.
            for i in range(180 - half, 180 + half + 1):
                ranges[i % 360] = front
        m.ranges = ranges
        self.scan_pub.publish(m)

    def odom(self):
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(o)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)


def main():
    rclpy.init()
    n = FakeRover()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
