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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('leo_nav2_exploration')
    params = os.path.join(share, 'config', 'real', 'explore.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('progress_timeout', default_value='60.0'),
        DeclareLaunchArgument('min_frontier_size', default_value='0.5'),
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[
                params,
                {'progress_timeout': LaunchConfiguration('progress_timeout')},
                {'min_frontier_size': LaunchConfiguration('min_frontier_size')},
            ],
        ),
    ])
