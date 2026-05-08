import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LeoIntelligence(Node):
    def __init__(self):
        super().__init__('leo_intelligence')

        # Use sim time if available
        self.declare_parameter('use_sim_time', False)

        # Subscribing to the scan topic within the namespace
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Publisher for movement
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def scan_callback(self, msg):
        # Example logic: Stop if something is closer than 0.5m
        min_range = min(msg.ranges)
        drive_msg = Twist()

        if min_range < 0.5:
            self.get_logger().warn(f"Obstacle detected at {min_range:.2f}m! Stopping.")
            drive_msg.linear.x = 0.0
        else:
            drive_msg.linear.x = 0.2

        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeoIntelligence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()