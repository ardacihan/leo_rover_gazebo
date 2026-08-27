#!/usr/bin/env python3
"""Keyboard teleop for the demo runs, with a robot switch.

Self-contained on purpose. `scripts/teleop_wsl.sh` calls
`ros2 run leo_rover_control keyboard_control`, but that package now ships only
the console-script shim in `install/` -- its `keyboard_control.py` is not in
the tree, so the entry point raises on import. A demo must not depend on that.

    W / S   forward / back        A / D   turn left / right
    SPACE   stop                  1 / 2   drive leo1 / leo2
    - / =   slower / faster       Q       quit

Publishes geometry_msgs/Twist on /leoN/cmd_vel -- the topic the Gazebo
diff-drive plugin listens on, and the same one Nav2's controller_server is
remapped to, so teleop and autonomy share a channel and never both drive.

Usage (in an interactive terminal, inside the container):
    python3 demo_teleop.py [num_robots]
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

LIN_STEP = 0.20      # m/s per key press, capped at MAX_LIN
ANG_STEP = 0.60      # rad/s per key press, capped at MAX_ANG
MAX_LIN = 0.60
MAX_ANG = 1.80


class DemoTeleop(Node):
    def __init__(self, num_robots):
        super().__init__('demo_teleop')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pubs = {i + 1: self.create_publisher(Twist, f'/leo{i + 1}/cmd_vel', 10)
                     for i in range(num_robots)}
        self.active = 1
        self.scale = 1.0
        self.lin = 0.0
        self.ang = 0.0

    def send(self):
        msg = Twist()
        msg.linear.x = self.lin * self.scale
        msg.angular.z = self.ang * self.scale
        self.pubs[self.active].publish(msg)

    def stop_all(self):
        """Every rover, not just the active one -- switching robots while the
        previous one is rolling would otherwise leave it driving into a wall."""
        for pub in self.pubs.values():
            pub.publish(Twist())


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    num_robots = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init()
    node = DemoTeleop(num_robots)
    print(__doc__.split('Usage')[0])
    print(f'driving leo{node.active} of {num_robots}. Ctrl+C or Q to quit.\n')

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            key = sys.stdin.read(1).lower()
            if key in ('q', '\x03'):
                break
            elif key == 'w':
                node.lin = clamp(node.lin + LIN_STEP, -MAX_LIN, MAX_LIN)
            elif key == 's':
                node.lin = clamp(node.lin - LIN_STEP, -MAX_LIN, MAX_LIN)
            elif key == 'a':
                node.ang = clamp(node.ang + ANG_STEP, -MAX_ANG, MAX_ANG)
            elif key == 'd':
                node.ang = clamp(node.ang - ANG_STEP, -MAX_ANG, MAX_ANG)
            elif key == ' ':
                node.lin = node.ang = 0.0
            elif key in ('-', '_'):
                node.scale = clamp(node.scale - 0.1, 0.2, 2.0)
            elif key in ('=', '+'):
                node.scale = clamp(node.scale + 0.1, 0.2, 2.0)
            elif key.isdigit() and int(key) in node.pubs:
                node.lin = node.ang = 0.0
                node.stop_all()
                node.active = int(key)
            else:
                continue
            node.send()
            sys.stdout.write(
                f'\rleo{node.active}  v={node.lin * node.scale:+.2f} m/s  '
                f'w={node.ang * node.scale:+.2f} rad/s  scale={node.scale:.1f}   ')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_all()
        print('\nstopped.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
