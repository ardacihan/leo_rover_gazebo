#!/usr/bin/env python3
"""Record each rover's map-frame trajectory to CSV: t,robot,x,y.

Usage (inside the container):
    python3 traj_recorder.py <robots-csv> <out.csv> [period_sec]
    e.g. python3 traj_recorder.py leo1,leo2 /ros2_ws/out/traj.csv 2.0
"""

import sys

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TrajRecorder(Node):
    def __init__(self, robots, path, period):
        super().__init__('traj_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.robots = robots
        self.f = open(path, 'w')
        self.f.write('t,robot,x,y\n')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(period, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for r in self.robots:
            try:
                tf = self.buf.lookup_transform(
                    'map', f'{r}/base_link', rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            p = tf.transform.translation
            self.f.write(f'{now:.1f},{r},{p.x:.3f},{p.y:.3f}\n')
        self.f.flush()


def main():
    robots = (sys.argv[1].split(',') if len(sys.argv) > 1
              else ['leo1', 'leo2'])
    path = sys.argv[2] if len(sys.argv) > 2 else '/ros2_ws/traj.csv'
    period = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    rclpy.init()
    node = TrajRecorder(robots, path, period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            node.f.flush()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
