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

    if robot_namespace and not robot_namespace.startswith('/'):
      robot_namespace = '/' + robot_namespace

    self.cmd_vel_topic = f'{robot_namespace}/cmd_vel'
    self.twist = Twist()
    self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
    self.create_timer(0.1, self._publish_twist)

    self.linear_speed = 0.5
    self.angular_speed = 1.0

    self.get_logger().info('Keyboard teleop -> %s' % self.cmd_vel_topic)
    self.get_logger().info('W/S forward/back  A/D turn  Space stop  Q quit')

  def _publish_twist(self):
    self.publisher.publish(self.twist)

  def get_key(self):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

  def run(self):
    while rclpy.ok():
      key = self.get_key()
      if key in ('w', 'W'):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
      elif key in ('s', 'S'):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = 0.0
      elif key in ('a', 'A'):
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed
      elif key in ('d', 'D'):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.angular_speed
      elif key == ' ':
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
      elif key in ('q', 'Q'):
        break
      rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
  rclpy.init(args=args)
  robot_ns = 'leo1'
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
