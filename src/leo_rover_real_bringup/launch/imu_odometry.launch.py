"""Gyro-fused odometry for Rover 4.

Replaces `leo-nav-bridge.service`, which cannot be told to stop broadcasting
`odom -> base_footprint`. Stop that service before launching this, or the
transform will have two competing owners.

    sudo systemctl stop leo-nav-bridge
    ros2 launch leo_rover_real_bringup imu_odometry.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("leo_rover_real_bringup")
    ekf_params = os.path.join(package_dir, "config", "ekf_params.yaml")

    start_ekf = LaunchConfiguration("start_ekf")
    firmware_imu_topic = LaunchConfiguration("firmware_imu_topic")
    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")
    firmware_cmd_topic = LaunchConfiguration("firmware_cmd_topic")

    return LaunchDescription([
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument(
            "firmware_imu_topic", default_value="/rob_2/firmware/imu"
        ),
        DeclareLaunchArgument(
            "wheel_odom_topic", default_value="/rob_2/firmware/wheel_odom"
        ),
        DeclareLaunchArgument(
            "firmware_cmd_topic", default_value="/rob_2/cmd_vel"
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="firmware_relay.py",
            name="firmware_relay",
            parameters=[{
                "wheel_odom_topic": wheel_odom_topic,
                "firmware_cmd_topic": firmware_cmd_topic,
                # The EKF owns the transform whenever it runs.
                "publish_odom_tf": False,
            }],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="imu_bridge.py",
            name="imu_bridge",
            parameters=[{
                "firmware_imu_topic": firmware_imu_topic,
                "odom_topic": "/wheel_odom",
                "output_topic": "/imu/data_raw",
            }],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_params],
            condition=IfCondition(start_ekf),
            output="screen",
            emulate_tty=True,
        ),
    ])
