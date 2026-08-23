#!/usr/bin/env python3
"""Log ground-truth, odometry-only and SLAM poses to one CSV.

All three live in the same world frame: the sim spawns the rover at the origin,
scripts/sim_realism_odom.py seeds its integration on the true starting pose, and
slam_toolbox anchors ``map`` on the first scan. So the columns are directly
comparable and the SLAM absolute trajectory error is just the difference.

Columns: t, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, slam_x, slam_y, slam_yaw

Usage (inside the sim container):
    python3 /ros2_ws/scripts/pose_error_recorder.py <out.csv> [period_sec]
"""

import math
import sys

import rclpy
import rclpy.time
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseErrorRecorder(Node):

    def __init__(self, path, period):
        super().__init__('pose_error_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.f = open(path, 'w')
        self.f.write('t,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,'
                     'slam_x,slam_y,slam_yaw\n')
        self.gt = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/leo1/odom', self.on_gt,
                                 qos_profile_sensor_data)
        self.create_timer(period, self.tick)

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def lookup(self, parent, child):
        try:
            tf = self.buf.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))

    def tick(self):
        if self.gt is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        odom = self.lookup('leo1/odom', 'leo1/base_link')
        slam = self.lookup('map', 'leo1/base_link')
        cells = [f'{now:.2f}'] + [f'{v:.4f}' for v in self.gt]
        for pose in (odom, slam):
            cells += ([f'{v:.4f}' for v in pose] if pose else ['', '', ''])
        self.f.write(','.join(cells) + '\n')
        self.f.flush()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/pose_error.csv'
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rclpy.init()
    node = PoseErrorRecorder(path, period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            node.f.flush()
            node.f.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
