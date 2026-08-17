"""Start the existing Gazebo simulator with a repeatable two-room doorway fixture."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    overlay_share = get_package_share_directory('leo_nav2_exploration')
    gazebo_share = get_package_share_directory('leo_rover_gazebo')
    fixture = os.path.join(overlay_share, 'models', 'doorway_fixture', 'model.sdf')
    scenario = os.path.join(overlay_share, 'config', 'sim', 'doorway_goals.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'two_robots.launch.py')),
        launch_arguments={
            'world': 'empty',
            'leo1_pose': '-1.50,0.0,0.20,0.0,0.0,0.0',
            'leo2_pose': '20.0,20.0,0.20,0.0,0.0,0.0',
        }.items(),
    )

    spawn_fixture = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_doorway_fixture',
                arguments=[
                    '-name', 'leo_nav2_doorway_fixture',
                    '-file', fixture,
                    '-x', '0.0', '-y', '0.0', '-z', '0.0',
                ],
                output='screen',
            )
        ],
    )

    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(overlay_share, 'launch', 'sim_navigation.launch.py')
                ),
                launch_arguments={
                    'start_slam': 'true',
                    'enable_voxel': LaunchConfiguration('enable_voxel'),
                    'navigation_start_delay': '3.0',
                }.items(),
            )
        ],
    )

    regression = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='leo_nav2_exploration',
                executable='doorway_regression',
                name='doorway_regression',
                arguments=['--scenario', scenario, '--output', LaunchConfiguration('result_file')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('run_regression')),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('enable_voxel', default_value='true'),
            DeclareLaunchArgument(
                'run_regression',
                default_value='false',
                description='Manual goals first; enable only after Nav2 and SLAM are visibly stable.',
            ),
            DeclareLaunchArgument(
                'result_file',
                default_value='/tmp/leo_nav2_doorway_regression.json',
            ),
            gazebo,
            spawn_fixture,
            navigation,
            regression,
        ]
    )
