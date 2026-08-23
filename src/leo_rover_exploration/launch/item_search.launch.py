import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('leo_rover_exploration')
    default_params = os.path.join(pkg, 'config', 'frontier_explorer_leo1.yaml')
    default_markers = os.path.join(pkg, 'config', 'mock_markers_leo_world.yaml')

    params_file = LaunchConfiguration('params_file')
    map_save_path = LaunchConfiguration('map_save_path')
    use_mock_detector = LaunchConfiguration('use_mock_detector')
    markers_file = LaunchConfiguration('markers_file')

    explorer = Node(
        package='leo_rover_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'map_save_path': map_save_path}],
    )

    mock_detector = Node(
        package='leo_rover_exploration',
        executable='mock_aruco_detector',
        name='mock_aruco_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_mock_detector),
        parameters=[{
            'use_sim_time': True,
            'markers_file': markers_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Explorer parameter file'),
        DeclareLaunchArgument(
            'map_save_path', default_value='/ros2_ws/maps/explored_map',
            description='Base path (no extension) for the saved map'),
        DeclareLaunchArgument(
            'use_mock_detector', default_value='true',
            description='Start the mock ArUco detector (until the real one)'),
        DeclareLaunchArgument(
            'markers_file', default_value=default_markers,
            description='Ground-truth marker poses for the mock detector'),
        explorer,
        mock_detector,
    ])
