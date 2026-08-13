#!/usr/bin/env python3

"""Publish the Leo firmware IMU as a bias-corrected sensor_msgs/Imu.

The firmware publishes `leo_msgs/Imu` on a namespaced topic, while every
standard consumer (imu_filter, robot_localization) expects `sensor_msgs/Imu` on
`/imu/data_raw`. Nothing bridged the two on Rover 4, which is why `/imu/data`
and `/imu/data_raw` were silent and the IMU went entirely unused.
"""

import math

import rclpy
from leo_msgs.msg import Imu as LeoImu
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from imu_calibration import GyroBiasEstimator


class ImuBridge(Node):
    """Convert the firmware IMU, removing the stationary gyro offset."""

    def __init__(self):
        super().__init__("imu_bridge")
        self.declare_parameter("firmware_imu_topic", "/rob_2/firmware/imu")
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("output_topic", "/imu/data_raw")
        self.declare_parameter("frame_id", "imu_frame")
        self.declare_parameter("settle_samples", 200)
        # Rover 4 measured gyro_z sd 0.00088 rad/s and |accel| sd 0.023 m/s^2
        # while stationary. These variances are deliberately looser so the EKF
        # does not over-trust a sensor whose bias can drift with temperature.
        self.declare_parameter("angular_velocity_variance", 1.0e-4)
        self.declare_parameter("linear_acceleration_variance", 1.0e-2)
        self.declare_parameter("report_period", 10.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.angular_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        self.linear_variance = float(
            self.get_parameter("linear_acceleration_variance").value
        )
        self.estimator = GyroBiasEstimator(
            settle_samples=int(self.get_parameter("settle_samples").value)
        )
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.published = 0

        self.publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data
        )
        self.create_subscription(
            LeoImu, str(self.get_parameter("firmware_imu_topic").value),
            self._imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._odom_callback, qos_profile_sensor_data
        )
        self.create_timer(
            float(self.get_parameter("report_period").value), self._report
        )

    def _odom_callback(self, msg):
        self.linear_speed = float(msg.twist.twist.linear.x)
        self.angular_speed = float(msg.twist.twist.angular.z)

    def _imu_callback(self, msg):
        self.estimator.update(
            msg.gyro_z, self.linear_speed, self.angular_speed
        )
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.angular_velocity.x = float(msg.gyro_x)
        out.angular_velocity.y = float(msg.gyro_y)
        out.angular_velocity.z = self.estimator.correct(msg.gyro_z)
        out.linear_acceleration.x = float(msg.accel_x)
        out.linear_acceleration.y = float(msg.accel_y)
        out.linear_acceleration.z = float(msg.accel_z)
        # No magnetometer, so orientation is unknown. A -1 leading covariance
        # is the REP-145 signal telling consumers to ignore that field rather
        # than treat an identity quaternion as a real measurement.
        out.orientation_covariance[0] = -1.0
        for index in (0, 4, 8):
            out.angular_velocity_covariance[index] = self.angular_variance
            out.linear_acceleration_covariance[index] = self.linear_variance
        self.publisher.publish(out)
        self.published += 1

    def _report(self):
        state = "settled" if self.estimator.ready else "settling"
        self.get_logger().info(
            f"imu bridge {state}: gyro_z bias={self.estimator.bias:+.5f} rad/s "
            f"({math.degrees(self.estimator.bias)*60.0:+.1f} deg/min) from "
            f"{self.estimator.samples} stationary samples; "
            f"{self.published} messages republished"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
