#!/usr/bin/env python3
"""
Broadcast odom -> base_footprint transforms for SLAM.

Prefer Gazebo ground-truth /model/leoN/pose so TF does not drift when wheels
slip against walls. Fall back to /leoN/odom if model pose is unavailable.

TF stamps are taken from /leoN/scan (then odom) so slam_toolbox message filters
can look up transforms at scan time.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def quat_inverse(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    return Quaternion(
        x=q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        y=q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        z=q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
        w=q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    )


def quat_rotate(q: Quaternion, vx: float, vy: float, vz: float):
    qv = Quaternion(x=vx, y=vy, z=vz, w=0.0)
    q_inv = quat_inverse(q)
    rotated = quat_multiply(quat_multiply(q, qv), q_inv)
    return rotated.x, rotated.y, rotated.z


class OdomTfBroadcaster(Node):
    ROBOTS = ("leo1", "leo2")

    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.initial_model_poses = {}
        self.latest_model_poses = {}
        self.latest_odom_poses = {}
        self.latest_stamps = {}
        self.use_model_pose = {}

        for robot_name in self.ROBOTS:
            self.create_subscription(
                Pose,
                f"/model/{robot_name}/pose",
                lambda msg, name=robot_name: self.model_pose_cb(name, msg),
                10,
            )
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom",
                lambda msg, name=robot_name: self.odom_cb(name, msg),
                10,
            )
            self.create_subscription(
                LaserScan,
                f"/{robot_name}/scan",
                lambda msg, name=robot_name: self.scan_cb(name, msg),
                10,
            )

        self.create_timer(0.05, self.timer_cb)

    def model_pose_cb(self, robot_name: str, msg: Pose):
        if robot_name not in self.initial_model_poses:
            self.initial_model_poses[robot_name] = msg
            self.get_logger().info(
                f"Using /model/{robot_name}/pose for {robot_name}/odom TF"
            )
        self.latest_model_poses[robot_name] = msg
        self.use_model_pose[robot_name] = True
        self._publish_tf(robot_name)

    def odom_cb(self, robot_name: str, msg: Odometry):
        self.latest_odom_poses[robot_name] = msg.pose.pose
        self.latest_stamps[robot_name] = msg.header.stamp
        if robot_name not in self.use_model_pose:
            self._publish_tf(robot_name, msg.header.stamp)

    def scan_cb(self, robot_name: str, msg: LaserScan):
        self.latest_stamps[robot_name] = msg.header.stamp
        self._publish_tf(robot_name, msg.header.stamp)

    def timer_cb(self):
        for robot_name in self.ROBOTS:
            if robot_name in self.latest_stamps:
                self._publish_tf(robot_name, self.latest_stamps[robot_name])

    def _publish_tf(self, robot_name: str, stamp=None):
        transform = self._make_transform(robot_name)
        if transform is None:
            return

        if stamp is None:
            stamp = self.latest_stamps.get(robot_name)
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(transform)

    def _make_transform(self, robot_name: str):
        transform = TransformStamped()
        transform.header.frame_id = f"{robot_name}/odom"
        transform.child_frame_id = f"{robot_name}/base_footprint"

        if self.use_model_pose.get(robot_name) and robot_name in self.latest_model_poses:
            if robot_name not in self.initial_model_poses:
                return None

            msg = self.latest_model_poses[robot_name]
            origin = self.initial_model_poses[robot_name]
            q_origin_inv = quat_inverse(origin.orientation)

            dx = msg.position.x - origin.position.x
            dy = msg.position.y - origin.position.y
            dz = msg.position.z - origin.position.z
            tx, ty, tz = quat_rotate(q_origin_inv, dx, dy, dz)

            transform.transform.translation.x = tx
            transform.transform.translation.y = ty
            transform.transform.translation.z = tz
            transform.transform.rotation = quat_multiply(q_origin_inv, msg.orientation)
            return transform

        if robot_name in self.latest_odom_poses:
            pose = self.latest_odom_poses[robot_name]
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            return transform

        return None


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
