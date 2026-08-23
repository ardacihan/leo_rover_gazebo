#!/usr/bin/env python3
"""Degrade Gazebo's perfect IMU into something a real MEMS gyro would produce.

Gazebo's `imu` sensor derives orientation from ground truth and its angular
velocity is noise-free. Fusing either as-is would hand the EKF the answer and
make any yaw-drift result meaningless -- the same trap as scoring SLAM on the
simulator's ground-truth odometry.

This republishes `/leo1/imu/data` with:

  * a constant gyro bias drawn once per run (a real device's turn-on bias
    repeatability), plus a slow random walk (in-run bias instability);
  * white noise on the rate;
  * the orientation quaternion left untouched but *not intended for use* --
    the EKF config fuses angular velocity only.

Defaults correspond to a decent consumer MEMS gyro (ICM-42688 class):
bias stability ~20 deg/hr, noise density ~0.008 deg/s/sqrt(Hz).

Usage (inside the container):
    python3 sim_realism_imu.py --ros-args -p use_sim_time:=true -p seed:=1
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

DEG = math.pi / 180.0


class ImuDegrader(Node):

    def __init__(self):
        super().__init__('sim_realism_imu')
        self.declare_parameter('input_topic', '/leo1/imu/data')
        self.declare_parameter('output_topic', '/leo1/imu/data_real')
        # deg/hr of bias stability -> rad/s
        self.declare_parameter('bias_stability_dph', 20.0)
        # turn-on bias, deg/s
        self.declare_parameter('turn_on_bias_dps', 0.15)
        # deg/s/sqrt(Hz)
        self.declare_parameter('noise_density_dps_rthz', 0.008)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('seed', 0)

        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(seed if seed else None)

        rate = float(self.get_parameter('rate_hz').value)
        self.bias = float(self.get_parameter('turn_on_bias_dps').value) * DEG \
            * self.rng.normal()
        # Bias instability as a random walk increment per sample.
        self.walk = (float(self.get_parameter('bias_stability_dph').value) * DEG / 3600.0) \
            / math.sqrt(max(rate, 1.0))
        self.sigma = float(self.get_parameter('noise_density_dps_rthz').value) * DEG \
            * math.sqrt(max(rate, 1.0))

        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Imu, out, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('input_topic').value,
            self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f'IMU degrader: turn-on bias {self.bias / DEG:.3f} deg/s, '
            f'walk {self.walk:.2e} rad/s/sample, sigma {self.sigma:.2e} rad/s -> {out}')

    def on_imu(self, msg):
        self.bias += self.walk * self.rng.normal()
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        # Flag orientation as unusable so nothing downstream is tempted by it;
        # the EKF is configured to fuse angular velocity only.
        out.orientation_covariance[0] = -1.0
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        out.angular_velocity.z = (msg.angular_velocity.z + self.bias
                                  + self.sigma * self.rng.normal())
        out.angular_velocity_covariance = [0.0] * 9
        out.angular_velocity_covariance[8] = float(self.sigma ** 2 + 1e-6)
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = [0.0] * 9
        for i in (0, 4, 8):
            out.linear_acceleration_covariance[i] = 0.05
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuDegrader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
