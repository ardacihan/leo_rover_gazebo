import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('leo_rover_exploration')
    default_params = os.path.join(pkg, 'config', 'frontier_explorer_leo1.yaml')

    params_file = LaunchConfiguration('params_file')
    map_save_path = LaunchConfiguration('map_save_path')

    explorer = Node(
        package='leo_rover_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'map_save_path': map_save_path}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Explorer parameter file'),
        DeclareLaunchArgument(
            'map_save_path', default_value='/ros2_ws/maps/explored_map',
            description='Base path (no extension) for the saved map'),
        explorer,
    ])
