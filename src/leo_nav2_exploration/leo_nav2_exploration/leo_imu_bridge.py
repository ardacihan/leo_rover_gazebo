"""Republish the Leo Rover firmware IMU as `sensor_msgs/Imu`.

The rover's firmware publishes `leo_msgs/Imu` -- a flat struct of gyro and
accelerometer floats with no orientation and no covariances. `robot_localization`
only accepts `sensor_msgs/Imu`, so without this bridge the EKF has no gyro and
the yaw channel falls back to wheel odometry, which on a skid-steer chassis is
the single worst-measured quantity on the robot.

Two conventions this node sets deliberately:

* `orientation_covariance[0] = -1`, the ROS signal for "no orientation in this
  message". The firmware IMU has no magnetometer and reports no fused attitude;
  publishing a zero quaternion without this flag would let a filter treat
  "facing along +x" as a measurement.
* Only the **z gyro** is trusted downstream. The EKF config fuses yaw rate
  alone. The other axes are still published, honestly, for anyone who wants
  them, but the covariances say what they are worth.

Gyro bias is estimated during an initial stationary window and subtracted. A
MEMS gyro's bias is the dominant error term over a mapping run: 0.5 deg/s of
uncorrected bias is 30 degrees of heading after one minute, which is worse than
the wheel odometry it replaces. `calibration_samples: 0` disables it if the
rover cannot be held still at startup.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

try:
    from leo_msgs.msg import Imu as LeoImu
except ImportError:  # pragma: no cover - only importable on the rover
    LeoImu = None


class LeoImuBridge(Node):

    def __init__(self):
        super().__init__('leo_imu_bridge')

        p = self.declare_parameter
        p('input_topic', '/firmware/imu')
        p('output_topic', '/imu/data')
        p('frame_id', 'imu_link')
        # ICM-42688 / MPU-class noise, as covariance (rad/s)^2 and (m/s^2)^2.
        # Deliberately pessimistic: an EKF that trusts a MEMS gyro too much
        # tracks its bias walk instead of correcting it.
        p('gyro_variance', 0.0004)
        p('accel_variance', 0.04)
        p('calibration_samples', 200)
        p('use_message_stamp', True)

        g = lambda n: self.get_parameter(n).value
        self.frame_id = g('frame_id')
        self.gyro_var = float(g('gyro_variance'))
        self.accel_var = float(g('accel_variance'))
        self.cal_target = int(g('calibration_samples'))
        self.use_msg_stamp = bool(g('use_message_stamp'))

        self.bias = [0.0, 0.0, 0.0]
        self.cal_sum = [0.0, 0.0, 0.0]
        self.cal_n = 0
        self.calibrated = self.cal_target <= 0
        self.count = 0

        if LeoImu is None:
            raise RuntimeError(
                'leo_msgs is not installed; this bridge only runs on the rover '
                '(or wherever leo_msgs is built)')

        self.pub = self.create_publisher(Imu, g('output_topic'), 10)
        self.create_subscription(LeoImu, g('input_topic'), self.on_imu,
                                 qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"leo_imu_bridge: {g('input_topic')} (leo_msgs/Imu) -> "
            f"{g('output_topic')} (sensor_msgs/Imu), frame {self.frame_id}, "
            f'bias calibration over {self.cal_target} stationary samples')

    def on_imu(self, msg):
        self.count += 1
        gyro = [float(msg.gyro_x), float(msg.gyro_y), float(msg.gyro_z)]

        if not self.calibrated:
            for i in range(3):
                self.cal_sum[i] += gyro[i]
            self.cal_n += 1
            if self.cal_n >= self.cal_target:
                self.bias = [s / self.cal_n for s in self.cal_sum]
                self.calibrated = True
                self.get_logger().info(
                    'gyro bias: '
                    f'x={self.bias[0]:+.5f} y={self.bias[1]:+.5f} '
                    f'z={self.bias[2]:+.5f} rad/s '
                    f'(z = {math.degrees(self.bias[2]):+.3f} deg/s). '
                    'If the rover was moving during startup this is wrong -- '
                    'restart it stationary.')
            return  # do not publish biased samples

        out = Imu()
        out.header.frame_id = self.frame_id
        if self.use_msg_stamp and (msg.stamp.sec or msg.stamp.nanosec):
            out.header.stamp = msg.stamp
        else:
            out.header.stamp = self.get_clock().now().to_msg()

        out.angular_velocity.x = gyro[0] - self.bias[0]
        out.angular_velocity.y = gyro[1] - self.bias[1]
        out.angular_velocity.z = gyro[2] - self.bias[2]
        out.linear_acceleration.x = float(msg.accel_x)
        out.linear_acceleration.y = float(msg.accel_y)
        out.linear_acceleration.z = float(msg.accel_z)

        # No fused attitude on this sensor. -1 is the ROS convention for
        # "orientation absent"; every well-behaved consumer honours it.
        out.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            out.angular_velocity_covariance[i] = self.gyro_var
            out.linear_acceleration_covariance[i] = self.accel_var
        self.pub.publish(out)

    def report(self):
        state = 'calibrated' if self.calibrated else f'calibrating {self.cal_n}/{self.cal_target}'
        self.get_logger().info(f'{self.count} firmware IMU messages, {state}')


def main(args=None):
    rclpy.init(args=args)
    node = LeoImuBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
