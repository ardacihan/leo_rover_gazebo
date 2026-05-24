import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_gazebo       = get_package_share_directory('leo_rover_gazebo')

    params_file = os.path.join(pkg_gazebo, 'config', 'nav2_params_leo.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items(),
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel_smoothed', '/leo1/cmd_vel'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([nav2, cmd_vel_relay])