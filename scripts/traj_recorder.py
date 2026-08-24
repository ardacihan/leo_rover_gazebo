#!/usr/bin/env python3
"""Record each rover's map-frame trajectory to CSV: t,robot,x,y.

Usage (inside the container):
    python3 traj_recorder.py <robots-csv> <out.csv> [period_sec] [frame]
    e.g. python3 traj_recorder.py leo1,leo2 /ros2_ws/out/traj.csv 2.0 leo1/map

`frame` defaults to 'map'. Without multirobot_map_merge no global 'map' frame
exists and every lookup fails silently, leaving an empty trajectory file; under
tag alignment pass leo1/map, the frame the shared map is published in.
"""

import sys

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TrajRecorder(Node):
    def __init__(self, robots, path, period, frame='map'):
        super().__init__('traj_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.robots = robots
        self.frame = frame
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
                    self.frame, f'{r}/base_link', rclpy.time.Time(),
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
    frame = sys.argv[4] if len(sys.argv) > 4 else 'map'
    rclpy.init()
    node = TrajRecorder(robots, path, period, frame)
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
