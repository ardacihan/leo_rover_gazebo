#!/usr/bin/env python3
"""Publish a deterministic AMCL initial pose without ros2cli."""

import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    node = rclpy.create_node('validation_initial_pose')
    qos = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', qos)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.05
    message.pose.covariance[7] = 0.05
    message.pose.covariance[35] = 0.02
    for _ in range(10):
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'published initial pose ({x}, {y})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
