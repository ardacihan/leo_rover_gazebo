"""Namespaced SLAM for N Leo rovers (collaborative mapping).

One async slam_toolbox per robot, each under its own namespace so it
publishes /leo{i}/map in frame leo{i}/map and broadcasts leo{i}/map ->
leo{i}/odom. The drift-tuned slam_params_leo.yaml is reused verbatim; only
the per-robot frames and scan topic are overridden. multirobot_map_merge
(map_merge_leo.launch.py) fuses the per-robot maps into a global /map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    slam_params = os.path.join(
        get_package_share_directory('leo_rover_gazebo'),
        'config', 'slam_params_leo.yaml',
    )

    nodes = []
    for i in range(num_robots):
        ns = f'leo{i + 1}'
        overrides = {
            'use_sim_time': True,
            'odom_frame': f'{ns}/odom',
            'base_frame': f'{ns}/base_link',
            'map_frame': f'{ns}/map',
            'scan_topic': f'/{ns}/scan',
        }
        nodes.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[slam_params, overrides],
            # /tf and /tf_static are global; without this the namespaced node
            # would publish to /leo{i}/tf and the transform tree would break.
            # slam_toolbox hardcodes ABSOLUTE /map and /map_metadata, so the
            # namespace does not move them - remap explicitly, otherwise both
            # rovers' slam nodes clobber each other on a single /map and the
            # per-robot maps (/leo{i}/map) have no publisher at all.
            remappings=[
                ('/tf', '/tf'), ('/tf_static', '/tf_static'),
                ('/map', f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots', default_value='2',
            description='Number of rovers (leo1..leoN) to run SLAM for'),
        OpaqueFunction(function=launch_setup),
    ])
