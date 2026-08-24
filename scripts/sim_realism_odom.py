#!/usr/bin/env python3
"""Replace Gazebo's ground-truth odometry with a realistic wheel-odometry model.

The sim's ``ignition-gazebo-odometry-publisher-system`` reads the *true* model
pose out of the simulator and publishes it as ``odom -> base_link``. SLAM
therefore starts every scan match from a perfect prior, so any slam_toolbox
configuration looks good in sim and nothing predicts hardware behaviour. The
physical rover runs ``leo_rover_real_bringup/scripts/wheel_odom_tf.py``, which
integrates wheel twist only -- on a four-wheel skid-steer that accumulates a
large yaw error whenever the robot turns.

This node reproduces that. It differentiates the ground-truth pose to recover
the true body motion, corrupts it with a skid-steer error model, integrates the
result, and publishes the corrupted pose as the ``odom -> base_link`` TF plus a
matching Odometry message.

Error model, per integration step (true forward step ``ds``, true yaw step
``dth``):

    ds_m  = ds  * (1 + linear_scale)  + N(0, linear_noise * sqrt(|ds|))
    dth_m = dth * (1 + yaw_scale)     + N(0, yaw_noise   * sqrt(|dth|))
                                      + slip_per_metre * ds * N(0, 1)
                                      + bias * dt

``yaw_scale`` is the dominant term: a skid-steer rotates *less* than its wheel
kinematics predict, so wheel odometry over-reports yaw. The repository's own URDF
documents the size of this effect -- it inflates ``wheel_separation`` from the
physical 0.358 m to 0.537 m to cancel it, a 50 % correction.

Usage (inside the sim container):

    python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
        -p yaw_scale:=0.12 -p seed:=1
"""

import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RealisticOdom(Node):

    def __init__(self):
        super().__init__('sim_realism_odom')
        self.declare_parameter('input_topic', '/leo1/odom')
        self.declare_parameter('output_topic', '/leo1/odom_wheel_like')
        self.declare_parameter('odom_frame', 'leo1/odom')
        self.declare_parameter('base_frame', 'leo1/base_link')
        self.declare_parameter('publish_tf', True)
        # --- error model ---
        self.declare_parameter('yaw_scale', 0.12)        # systematic, skid-steer
        self.declare_parameter('linear_scale', 0.02)     # wheel-radius error
        self.declare_parameter('yaw_noise', 0.02)        # rad / sqrt(rad)
        self.declare_parameter('linear_noise', 0.01)     # m / sqrt(m)
        self.declare_parameter('slip_per_metre', 0.01)   # rad / m travelled
        self.declare_parameter('yaw_bias', 0.0)          # rad/s constant drift
        self.declare_parameter('seed', 0)
        # See on_odom(): false keeps the single-robot behaviour of
        # seeding on the true pose; true starts at (0, 0, 0) as real
        # wheel odometry does, which is required for an honest
        # two-rover alignment test.
        self.declare_parameter('zero_origin', False)

        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.yaw_scale = float(self.get_parameter('yaw_scale').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.yaw_noise = float(self.get_parameter('yaw_noise').value)
        self.linear_noise = float(self.get_parameter('linear_noise').value)
        self.slip_per_metre = float(self.get_parameter('slip_per_metre').value)
        self.yaw_bias = float(self.get_parameter('yaw_bias').value)
        self.rng = random.Random(int(self.get_parameter('seed').value))
        self.zero_origin = bool(self.get_parameter('zero_origin').value)

        self.prev = None          # (x, y, yaw, t) ground truth
        self.x = self.y = self.th = 0.0
        self.true_path = 0.0
        self.est_path = 0.0

        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value,
            self.on_odom, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f'realistic odom: yaw_scale={self.yaw_scale} '
            f'linear_scale={self.linear_scale} slip={self.slip_per_metre} '
            f'-> {self.output_topic} / TF {self.odom_frame}->{self.base_frame}')

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        gth = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev is None:
            self.prev = (gx, gy, gth, t)
            if self.zero_origin:
                # Wheel odometry on a real rover reads zero wherever it was
                # switched on; it has no idea where in the world that is. With
                # two rovers that distinction is the whole problem: seeding on
                # the true pose puts BOTH SLAM maps in the world frame, so the
                # leo2/map -> leo1/map transform the tag aligner is supposed to
                # recover is identity by construction and the rovers secretly
                # share a frame from the first scan. Starting at the origin
                # anchors each map on its own rover's start pose, which makes
                # the true transform the actual spawn offset.
                self.x = self.y = self.th = 0.0
            else:
                # Single-robot default, kept: map and odom share an origin,
                # which is what the pose-error tooling assumes.
                self.x, self.y, self.th = gx, gy, gth
            return

        px, py, pth, pt = self.prev
        dt = t - pt
        if dt <= 0.0 or dt > 1.0:
            self.prev = (gx, gy, gth, t)
            return

        # True incremental motion expressed in the robot body frame.
        dx_w, dy_w = gx - px, gy - py
        ds = math.hypot(dx_w, dy_w)
        # Sign of travel: forward if the world displacement agrees with heading.
        if ds > 1e-9 and (dx_w * math.cos(pth) + dy_w * math.sin(pth)) < 0.0:
            ds = -ds
        dth = wrap(gth - pth)

        self.true_path += abs(ds)

        # --- corrupt ---
        ds_m = ds * (1.0 + self.linear_scale)
        if ds != 0.0:
            ds_m += self.rng.gauss(0.0, self.linear_noise * math.sqrt(abs(ds)))
        dth_m = dth * (1.0 + self.yaw_scale) + self.yaw_bias * dt
        if dth != 0.0:
            dth_m += self.rng.gauss(0.0, self.yaw_noise * math.sqrt(abs(dth)))
        if ds != 0.0:
            dth_m += self.rng.gauss(0.0, self.slip_per_metre * abs(ds))

        # --- integrate (midpoint) ---
        mid = self.th + 0.5 * dth_m
        self.x += ds_m * math.cos(mid)
        self.y += ds_m * math.sin(mid)
        self.th = wrap(self.th + dth_m)
        self.est_path += abs(ds_m)

        self.prev = (gx, gy, gth, t)
        self.publish(msg.header.stamp, ds_m / dt, dth_m / dt)

    def publish(self, stamp, v, w):
        qz, qw = math.sin(self.th * 0.5), math.cos(self.th * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        self.pub.publish(odom)

        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def report(self):
        if self.prev is None:
            return
        gx, gy, gth, _ = self.prev
        self.get_logger().info(
            f'odom error: pos={math.hypot(self.x - gx, self.y - gy):.3f} m '
            f'yaw={math.degrees(wrap(self.th - gth)):+.1f} deg '
            f'after {self.true_path:.1f} m travelled')


def main():
    rclpy.init()
    node = RealisticOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
