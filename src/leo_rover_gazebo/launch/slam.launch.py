import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('leo_rover_gazebo'),
        'config', 'slam_params_leo.yaml',
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[slam_params],
            remappings=[('/scan', '/leo1/scan')],
        )
    ])