#!/usr/bin/env python3
"""Keyboard teleop for the PHYSICAL Leo Rover, through the safety chain.

    W / S   forward / back        A / D   turn left / right
    SPACE   stop                  - / =   slower / faster
    Q       quit (sends a stop first)

Publishes geometry_msgs/Twist on **/cmd_vel_request**, the head of the chain:

    teleop / explorer  ->  /cmd_vel_request
    safety_command_gate ->  /cmd_vel_raw
    collision_monitor   ->  /cmd_vel
    firmware_relay      ->  /rob_2/cmd_vel

Publishing straight to /cmd_vel bypasses the collision monitor, and the gate
audits that topic's publishers and closes when it sees one it does not know
(`allowed_cmd_vel_output_publishers` is collision_monitor and the leo_real
supervisor). Another operator's keyboard teleop on /cmd_vel is a documented
incident on this robot, so this node refuses those topics outright.

Two things the gate does that make teleop feel odd, and both are correct:

* **Commands go stale in 0.3 s** (`command_timeout`), so this publishes at
  20 Hz continuously, not once per key press. A key sets a velocity; the
  velocity persists until you change it or `idle_stop` expires.
* **Your speed request is capped**, at 0.10 m/s forward and 0.30 rad/s by
  default, and **reverse is blocked entirely** (`maximum_reverse_speed` is
  0.0 unless safe_mapping.launch.py was given another value). S will look
  like it does nothing. That is the gate, not this node.

Usage (on the rover, stack already up):
    python3 rover_teleop.py
    python3 rover_teleop.py --topic /cmd_vel --unsafe    # only with
                                                         # start_safety:=false
"""

import argparse
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RATE_HZ = 20.0
LIN_STEP = 0.02          # small steps: the gate caps at 0.10 m/s anyway
ANG_STEP = 0.10
MAX_LIN = 0.15           # velocity_smoother's validated ceiling
MAX_ANG = 0.60
BANNED = ('/cmd_vel_raw',)


class RoverTeleop(Node):
    def __init__(self, topic, idle_stop):
        super().__init__('rover_teleop')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.lin = 0.0
        self.ang = 0.0
        self.scale = 1.0
        self.idle_stop = idle_stop
        self.idle = 0.0
        self.lock = threading.Lock()
        # The gate drops a command older than 0.3 s, so the stream is the
        # contract -- a key press is only an edit to what is already streaming.
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(f'streaming Twist on {topic} at {RATE_HZ:.0f} Hz')

    def _tick(self):
        with self.lock:
            if self.idle_stop > 0.0:
                self.idle += 1.0 / RATE_HZ
                if self.idle >= self.idle_stop and (self.lin or self.ang):
                    self.lin = self.ang = 0.0
                    self.get_logger().warn(
                        f'no key for {self.idle_stop:.0f}s - stopping')
            msg = Twist()
            msg.linear.x = self.lin * self.scale
            msg.angular.z = self.ang * self.scale
        self.pub.publish(msg)

    def stop(self):
        with self.lock:
            self.lin = self.ang = 0.0
        self.pub.publish(Twist())


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/cmd_vel_request')
    ap.add_argument('--idle-stop', type=float, default=5.0,
                    help='seconds without a key press before zeroing; 0 to '
                         'disable')
    ap.add_argument('--unsafe', action='store_true',
                    help='required to publish anywhere but /cmd_vel_request')
    args = ap.parse_args()

    topic = args.topic.rstrip('/')
    if topic in BANNED:
        sys.exit(f'refusing {topic}: that is the gate\'s own output stage')
    if topic != '/cmd_vel_request' and not args.unsafe:
        sys.exit(
            f'refusing {topic} without --unsafe.\n'
            '/cmd_vel_request is the head of the safety chain. Publishing to\n'
            '/cmd_vel bypasses the collision monitor AND trips the gate\'s\n'
            'publisher audit, which closes the gate. Only pass --unsafe when\n'
            'the stack was launched with start_safety:=false.')

    rclpy.init()
    node = RoverTeleop(topic, args.idle_stop)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print(__doc__.split('Publishes')[0])
    print(f'publishing on {topic}. Ctrl+C or Q to quit.\n')
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            key = sys.stdin.read(1).lower()
            if key in ('q', '\x03'):
                break
            with node.lock:
                node.idle = 0.0
                if key == 'w':
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
                    node.scale = clamp(node.scale - 0.1, 0.2, 1.0)
                elif key in ('=', '+'):
                    node.scale = clamp(node.scale + 0.1, 0.2, 1.0)
                else:
                    continue
                lin, ang, scale = node.lin, node.ang, node.scale
            sys.stdout.write(
                f'\rrequest v={lin * scale:+.3f} m/s  w={ang * scale:+.2f} rad/s'
                f'  scale={scale:.1f}   (gate caps at 0.10 / 0.30)   ')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        print('\nstopped.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
