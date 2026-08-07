import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("leo_rover_real_bringup"),
        "config",
        "slam_params.yaml",
    )

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_static_transform",
            arguments=[
                "--x", lidar_x,
                "--y", lidar_y,
                "--z", lidar_z,
                "--yaw", lidar_yaw,
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "laser",
            ],
            output="screen",
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[params_file],
            output="screen",
        ),
    ])
