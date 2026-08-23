"""Collaborative map merging for the Leo rovers.

Runs the deterministic compositor that fuses the per-robot SLAM maps
(/leo1/map, /leo2/map) into a single global /map, and publishes the static
map -> leo{i}/map transforms that anchor every robot's own frame in the
common one.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (x, y, yaw) of each rover's map frame in the merged "map" frame.
#
# IDENTITY for every robot, NOT the spawn offsets: /leo{i}/odom comes from
# the Gazebo OdometryPublisher system, which reports WORLD-frame pose - so
# each robot's odom (and therefore slam map) frame is already anchored at
# the world origin regardless of where the robot spawned. (Verified
# 2026-07-13 by registering leo2's saved office map against leo1's: best-fit
# offset (-0.06 m, 0.00 m, 0.35 deg), not the (1.5, 0) spawn. The old
# spawn-offset TFs shifted every cross-robot conversion by 1.5 m and were
# the real source of the doubled walls/seams in earlier merged maps.)
INIT_POSES = {
    'leo1': (0.0, 0.0, 0.0),
    'leo2': (0.0, 0.0, 0.0),
}


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Custom deterministic compositor (see scripts/map_compositor.py). It uses
    # the same known offsets as the static TFs below, so the merged /map is
    # exactly aligned with every robot's pose - unlike multirobot_map_merge,
    # whose leading-slash init_pose params silently fail under our Humble build
    # and place the offset robot ~init metres wrong.
    ws_root = os.environ.get('ROS2_WS', '/ros2_ws')
    merge_node = ExecuteProcess(
        cmd=['python3', os.path.join(ws_root, 'scripts', 'map_compositor.py'),
             'leo1,leo2', '2.0'],
        output='screen',
    )

    static_tfs = []
    for ns, (x, y, yaw) in INIT_POSES.items():
        static_tfs.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_{ns}_map',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', '0',
                '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', f'{ns}/map',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        merge_node,
        *static_tfs,
    ])
