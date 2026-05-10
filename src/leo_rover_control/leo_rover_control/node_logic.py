# ~/PycharmProjects/leo_rover_gazebo/src/leo_rover_control/leo_rover_control/node_logic.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardControl(Node):
    def __init__(self, robot_namespace='leo1'):
        super().__init__('keyboard_control')

        # Check if namespace is provided and properly formatted
        if robot_namespace and not robot_namespace.startswith('/'):
            robot_namespace = '/' + robot_namespace

        self.cmd_vel_topic = f'{robot_namespace}/cmd_vel' if robot_namespace else 'cmd_vel'

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(f'=== Keyboard Teleop Ready ===')
        self.get_logger().info(f'Publishing to: {self.cmd_vel_topic}')
        self.get_logger().info('Controls:')
        self.get_logger().info('  w - forward')
        self.get_logger().info('  s - backward')
        self.get_logger().info('  a - turn left')
        self.get_logger().info('  d - turn right')
        self.get_logger().info('  space - stop')
        self.get_logger().info('  q - quit')

        self.linear_speed = 0.5
        self.angular_speed = 1.0

    def get_key(self):
        """Get keyboard input without waiting for enter"""
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = None
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        twist = Twist()

        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                self.get_logger().info(f'Moving forward: {self.linear_speed} m/s')
            elif key == 's':
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
                self.get_logger().info(f'Moving backward: {self.linear_speed} m/s')
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                self.get_logger().info(f'Turning left: {self.angular_speed} rad/s')
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
                self.get_logger().info(f'Turning right: {self.angular_speed} rad/s')
            elif key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Stopping')
            elif key == 'q':
                self.get_logger().info('Quitting...')
                break
            else:
                # Keep publishing the last command or stop if no key
                if key is None:
                    pass  # Continue publishing current twist
                else:
                    continue

            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)

    # Parse namespace from command line
    robot_ns = 'leo1'  # default
    for i, arg in enumerate(sys.argv):
        if arg == '--namespace' and i + 1 < len(sys.argv):
            robot_ns = sys.argv[i + 1]
            break

    node = KeyboardControl(robot_ns)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()