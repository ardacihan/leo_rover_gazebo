import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Keyboard control active. Use WASD keys to move robot. Press q to quit.')

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            if key == 'w':
                twist.linear.x = 0.5
                self.get_logger().info('Moving forward')
            elif key == 's':
                twist.linear.x = -0.5
                self.get_logger().info('Moving backward')
            elif key == 'a':
                twist.angular.z = 0.5
                self.get_logger().info('Turning left')
            elif key == 'd':
                twist.angular.z = -0.5
                self.get_logger().info('Turning right')
            elif key == 'q':
                self.get_logger().info('Quitting')
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()