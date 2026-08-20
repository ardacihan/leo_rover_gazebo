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
    return LaunchDescription(
        [
            DeclareLaunchArgument('start_slam', default_value='true'),
            # Depth camera as a costmap obstacle source. On by default: it is
            # what sees the table crossbars and chair legs the 2-D lidar plane
            # misses. Set false if the RealSense cannot keep up on the rover's
            # computer -- the lidar alone still builds the map.
            DeclareLaunchArgument('enable_voxel', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('use_respawn', default_value='false'),
            DeclareLaunchArgument('navigation_start_delay', default_value='3.0'),
            DeclareLaunchArgument('log_level', default_value='info'),
            include,
        ]
    )
