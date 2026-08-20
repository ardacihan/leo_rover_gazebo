"""Convenience wrapper for the root-level physical rover graph."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    package_share = get_package_share_directory('leo_nav2_exploration')
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'navigation_overlay.launch.py')),
        launch_arguments={
            'profile': 'real_root',
            'start_slam': LaunchConfiguration('start_slam'),
            'enable_voxel': LaunchConfiguration('enable_voxel'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'navigation_start_delay': LaunchConfiguration('navigation_start_delay'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )
    sensor_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'real_sensor_tf.launch.py')),
        launch_arguments={
            'publish_camera_tf': LaunchConfiguration('publish_camera_tf'),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('start_slam', default_value='true'),
            # enable_voxel:=true keeps the camera source in the local costmap
            # obstacle_layer (the flag name is historical). Enabled by default
            # since 2026-08-20: the rover-4 camera TF was calibrated against the
            # floor plane and the costmap band rejected 100% of floor points
            # (p99 floor height 16 mm vs the 60 mm min_obstacle_height).
            DeclareLaunchArgument('enable_voxel', default_value='true'),
            # The RealSense owns camera_link -> optical frames; this adds the
            # calibrated base_link -> camera_link mount. Set false if another
            # node (e.g. safe_mapping.launch.py) already publishes it.
            DeclareLaunchArgument('publish_camera_tf', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('use_respawn', default_value='false'),
            DeclareLaunchArgument('navigation_start_delay', default_value='3.0'),
            DeclareLaunchArgument('log_level', default_value='info'),
            sensor_tf,
            include,
        ]
    )
