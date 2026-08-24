"""Collaborative multi-robot frontier exploration + item search.

Launches one frontier_explorer per rover, namespaced /leo{i}. Each detects
frontiers on its OWN /leo{i}/map and coordinates via the shared claim topic +
TF tree. The coordination_mode launch arg switches the benchmark condition:

  coordinated  - distributed greedy allocation with proximity discount;
                 with item_search:=true also shares item confirmations and
                 camera-coverage claims between rovers
  independent  - uncoordinated baseline (each rover ignores the other)

item_search:=true additionally starts one mock ArUco detector per rover
(own-frame: LOS on /leo{i}/map, markers shifted common->own via TF) and sets
camera_coverage_target so the explorers run the SWEEPING/VERIFY states.

Node actions/services (navigate_to_pose, compute_path_to_pose, costmap clears,
slam_toolbox saves) resolve under each rover's namespace; /tf and /map are
shared globally.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    mode = LaunchConfiguration('coordination_mode').perform(context)
    item_search = LaunchConfiguration(
        'item_search').perform(context).lower() == 'true'
    markers_file = LaunchConfiguration('markers_file').perform(context)
    coverage_target = float(
        LaunchConfiguration('camera_coverage_target').perform(context))
    cfg_dir = os.path.join(
        get_package_share_directory('leo_rover_exploration'), 'config')

    # Shared registry/coverage claims only in the coordinated condition.
    share_claims = item_search and mode == 'coordinated'
    # Frame both rovers' positions are compared in. Under map_merge this
    # was the global 'map'; with tag alignment it is leo1/map, reached via
    # the transform alignment_tf_bridge publishes once alignment locks.
    common_frame = LaunchConfiguration('common_frame').perform(context)

    nodes = []
    for i in range(num_robots):
        ns = f'leo{i + 1}'
        params_file = os.path.join(cfg_dir, f'frontier_explorer_{ns}_multi.yaml')
        overrides = {'coordination_mode': mode,
                     'share_claims': share_claims,
                     'common_frame': common_frame}
        if item_search:
            overrides['camera_coverage_target'] = coverage_target
        nodes.append(Node(
            package='leo_rover_exploration',
            executable='frontier_explorer',
            name='frontier_explorer',
            namespace=ns,
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[params_file, overrides],
            # Share the global transform tree (namespaced tf listeners would
            # otherwise bind to /leo{i}/tf and lose map->base).
            remappings=[
                ('tf', '/tf'), ('tf_static', '/tf_static'),
                ('/tf', '/tf'), ('/tf_static', '/tf_static'),
            ],
        ))
        if item_search:
            nodes.append(Node(
                package='leo_rover_exploration',
                executable='mock_aruco_detector',
                name='mock_aruco_detector',
                namespace=ns,
                output='screen',
                respawn=True,
                respawn_delay=3.0,
                parameters=[{
                    'use_sim_time': True,
                    'markers_file': markers_file,
                    'camera_frame': f'{ns}/sensor_camera_link',
                    'map_frame': f'{ns}/map',
                    'common_frame': common_frame,
                    'map_topic': f'/{ns}/map',
                    'detection_topic': f'/{ns}/aruco_detections',
                }],
                remappings=[
                    ('tf', '/tf'), ('tf_static', '/tf_static'),
                    ('/tf', '/tf'), ('/tf_static', '/tf_static'),
                ],
            ))
    return nodes


def generate_launch_description():
    default_markers = os.path.join(
        get_package_share_directory('leo_rover_exploration'), 'config',
        'mock_markers_office_world.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument(
            'coordination_mode', default_value='coordinated',
            description='coordinated | independent'),
        DeclareLaunchArgument(
            'item_search', default_value='false',
            description='Enable camera sweep + mock detectors + item registry'),
        DeclareLaunchArgument('markers_file', default_value=default_markers),
        DeclareLaunchArgument(
            'common_frame', default_value='map',
            description="Frame peer poses are compared in. 'map' with "
                        "multirobot_map_merge; 'leo1/map' under tag "
                        'alignment, where no global map frame exists'),
        DeclareLaunchArgument(
            'camera_coverage_target', default_value='0.9',
            description='Wall fraction the camera must observe (item search)'),
        OpaqueFunction(function=launch_setup),
    ])
