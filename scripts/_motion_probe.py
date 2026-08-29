"""Sample commanded vs achieved velocity for one rover."""
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class P(Node):
    def __init__(self, ns):
        super().__init__('motion_probe')
        self.cmd = []
        self.od = []
        self.create_subscription(Twist, f'/{ns}/cmd_vel',
                                 lambda m: self.cmd.append((m.linear.x, m.angular.z)), 10)
        self.create_subscription(Odometry, f'/{ns}/odom',
                                 lambda m: self.od.append(
                                     (m.twist.twist.linear.x, m.twist.twist.angular.z,
                                      m.pose.pose.position.x, m.pose.pose.position.y)),
                                 qos_profile_sensor_data)


ns = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
rclpy.init()
n = P(ns)
end = n.get_clock().now().nanoseconds / 1e9 + secs
while n.get_clock().now().nanoseconds / 1e9 < end:
    rclpy.spin_once(n, timeout_sec=0.05)
print(f'{ns}: {len(n.cmd)} cmd_vel, {len(n.od)} odom over {secs}s')
if n.cmd:
    lx = [c[0] for c in n.cmd]
    az = [c[1] for c in n.cmd]
    print(f'  cmd linear.x  min {min(lx):+.3f} max {max(lx):+.3f} '
          f'mean {sum(lx) / len(lx):+.3f}  nonzero {sum(1 for v in lx if abs(v) > 1e-3)}/{len(lx)}')
    print(f'  cmd angular.z min {min(az):+.3f} max {max(az):+.3f} '
          f'mean {sum(az) / len(az):+.3f}  nonzero {sum(1 for v in az if abs(v) > 1e-3)}/{len(az)}')
if n.od:
    lx = [o[0] for o in n.od]
    az = [o[1] for o in n.od]
    print(f'  odom linear.x  min {min(lx):+.3f} max {max(lx):+.3f} mean {sum(lx) / len(lx):+.3f}')
    print(f'  odom angular.z min {min(az):+.3f} max {max(az):+.3f} mean {sum(az) / len(az):+.3f}')
    print(f'  odom pose start ({n.od[0][2]:.3f},{n.od[0][3]:.3f}) '
          f'end ({n.od[-1][2]:.3f},{n.od[-1][3]:.3f})')
