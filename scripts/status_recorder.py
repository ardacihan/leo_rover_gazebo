#!/usr/bin/env python3
"""Print explorer JSON status messages without using the ros2cli daemon."""

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('validation_status_recorder')
    node.create_subscription(
        String, '/frontier_explorer/status',
        lambda message: print(message.data, flush=True), 20)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
