"""Doorway regression wired to *this* repository's simulator launch.

The bundle's own ``sim_doorway_regression.launch.py`` includes
``two_robots.launch.py`` and passes ``world:='empty'`` plus ``leo1_pose`` /
``leo2_pose``. That launch file in this repo declares no launch arguments at
all -- it hardcodes ``husarion_office.sdf`` and ``num_robots = 1`` -- so every
one of those arguments is silently dropped. The fixture would be spawned at the
origin *inside the furnished office*, on top of existing geometry, and the rover
would start wherever the office launch puts it. The regression cannot pass.

This variant uses ``two_robots_gpu.launch.py``, which does honour ``world``,
``gui``, ``num_robots``, ``enable_camera`` and ``gt_odom_tf``, and places the
fixture relative to where that launch actually spawns leo1 (the world origin)
so the geometry in ``doorway_goals.yaml`` still holds: the door sits
``door_center_from_start`` = 1.5 m ahead of the rover.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    overlay_share = get_package_share_directory('leo_nav2_exploration')
    gazebo_share = get_package_share_directory('leo_rover_gazebo')
    fixture = os.path.join(overlay_share, 'models', 'doorway_fixture', 'model.sdf')
    scenario = os.path.join(overlay_share, 'config', 'sim', 'doorway_goals.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'two_robots_gpu.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'false',
            'num_robots': '1',
            'enable_camera': LaunchConfiguration('enable_camera'),
            'gt_odom_tf': LaunchConfiguration('gt_odom_tf'),
        }.items(),
    )

    # leo1 spawns at the world origin here, not at (-1.50, 0) as the bundle's
    # scenario assumes, so the fixture moves forward instead of the rover back.
    spawn_fixture = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_doorway_fixture',
                arguments=[
                    '-name', 'leo_nav2_doorway_fixture',
                    '-file', fixture,
                    '-x', '1.5', '-y', '0.0', '-z', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # With gt_odom_tf:=false the URDF's OdometryPublisher is off, so something
    # must own odom -> base_link or slam_toolbox's message filter queues every
    # scan and drops it ("discarding message because the queue is full"), map
    # never exists, and the regression dies waiting for map <- base_link.
    realism_odom = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '/ros2_ws/scripts/sim_realism_odom.py',
                     '--ros-args', '-p', 'use_sim_time:=true', '-p', 'seed:=1'],
                output='screen',
                condition=UnlessCondition(LaunchConfiguration('gt_odom_tf')),
            )
        ],
    )

    # slam_toolbox publishes no map -> odom until the rover has moved
    # minimum_travel_distance, so nudge it before anything waits on that TF.
    bootstrap_jog = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-lc',
                     'timeout 8 ros2 topic pub -r 5 /leo1/cmd_vel '
                     'geometry_msgs/msg/Twist "{linear: {x: 0.12}}" >/dev/null 2>&1; '
                     'timeout 6 ros2 topic pub -r 5 /leo1/cmd_vel '
                     'geometry_msgs/msg/Twist "{linear: {x: -0.12}}" >/dev/null 2>&1; '
                     'ros2 topic pub --once /leo1/cmd_vel '
                     'geometry_msgs/msg/Twist "{}" >/dev/null 2>&1'],
                output='screen',
            )
        ],
    )

    navigation = TimerAction(
        period=14.0,
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
        period=95.0,
        actions=[
            Node(
                package='leo_nav2_exploration',
                executable='doorway_regression',
                name='doorway_regression',
                arguments=['--scenario', scenario,
                           '--output', LaunchConfiguration('result_file')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('run_regression')),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('world', default_value='empty_with_plugins'),
            DeclareLaunchArgument('enable_camera', default_value='true'),
            DeclareLaunchArgument('gt_odom_tf', default_value='false'),
            DeclareLaunchArgument('enable_voxel', default_value='true'),
            DeclareLaunchArgument('run_regression', default_value='true'),
            DeclareLaunchArgument(
                'result_file',
                default_value='/tmp/leo_nav2_doorway_regression.json',
            ),
            gazebo,
            realism_odom,
            spawn_fixture,
            bootstrap_jog,
            navigation,
            regression,
        ]
    )
