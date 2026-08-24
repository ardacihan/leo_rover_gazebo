"""Autonomous frontier exploration on the physical rover.

    # in one terminal
    ros2 launch leo_nav2_exploration real_navigation.launch.py
    # once preflight passes, in another
    ros2 launch leo_nav2_exploration real_exploration.launch.py

Deliberately a separate launch from `real_navigation.launch.py`: the rover
should be *navigating correctly under your goals* before anything starts
choosing its own. Send a couple of goals from RViz first.

Uses `explore_lite`, not the bundle's `frontier_exploration_ros2`. The latter
declared "No more frontiers found" at 24% coverage in simulation once the SLAM
map's free space reached the edge of the occupancy grid.

Read `REAL_ROVER_DEPLOY.md` first — for a first hardware session,
`real_mapping.launch.py` and a joystick is the higher-probability path to a
finished map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    share = get_package_share_directory('leo_nav2_exploration')
    params = os.path.join(share, 'config', 'real', 'explore.yaml')

    # Two rovers on one LAN need one explorer each, in their own namespace.
    # explore.yaml's `costmap_topic: map` is *relative*, so the namespace moves
    # it to /rob_a/map for free -- but `robot_base_frame: base_footprint` is a
    # TF frame, not a topic, and namespaces do not touch TF. Left bare, both
    # rovers would look up the same frame and one of them would be driving on
    # the other's pose.
    ns = LaunchConfiguration('robot_ns').perform(context).strip('/')
    overrides = {
        'progress_timeout': LaunchConfiguration('progress_timeout'),
        'min_frontier_size': LaunchConfiguration('min_frontier_size'),
    }
    if ns:
        overrides['robot_base_frame'] = f'{ns}/base_footprint'

    return [Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace=ns or None,
        output='screen',
        parameters=[params, overrides],
        # /tf and /tf_static are global; a namespaced node would otherwise bind
        # to /rob_a/tf and lose the transform tree entirely.
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('progress_timeout', default_value='60.0'),
        DeclareLaunchArgument('min_frontier_size', default_value='0.5'),
        # Empty keeps the validated single-rover behaviour byte-for-byte.
        DeclareLaunchArgument(
            'robot_ns', default_value='',
            description='Rover namespace, e.g. rob_a. Empty = single-rover, '
                        'exactly as the 2026-08-20 field runs used it'),
        OpaqueFunction(function=_setup),
    ])
