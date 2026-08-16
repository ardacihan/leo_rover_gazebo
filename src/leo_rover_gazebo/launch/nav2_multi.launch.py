"""Namespaced Nav2 stacks for N Leo rovers (collaborative exploration).

Each rover gets its own Nav2 stack under /leo{i}. The per-robot param files
(nav2_params_leo{i}_multi.yaml) carry that robot's frames/topics and point
the global costmap's static layer at the *merged* /map, so every rover plans
against what all rovers have discovered. RewrittenYaml nests each file under
the robot namespace (root_key) so plain node keys like `controller_server:`
bind to the namespaced nodes - the standard nav2_bringup multi-robot pattern.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    cfg_dir = os.path.join(
        get_package_share_directory('leo_rover_gazebo'), 'config')

    nodes = []
    for i in range(num_robots):
        ns = f'leo{i + 1}'
        params_file = os.path.join(cfg_dir, f'nav2_params_{ns}_multi.yaml')
        configured = RewrittenYaml(
            source_file=params_file,
            root_key=ns,
            param_rewrites={'use_sim_time': 'true'},
            convert_types=True,
        )
        # /tf and /tf_static are shared globally; without these remaps the
        # namespaced nodes would use /leo{i}/tf and lose the transform tree.
        # cmd_vel is published absolutely by controller/behavior servers, so
        # steer it to the per-robot topic the diff-drive plugin listens on.
        common_remaps = [
            ('/tf', '/tf'), ('/tf_static', '/tf_static'),
            ('/cmd_vel', f'/{ns}/cmd_vel'),
        ]

        def lnode(pkg, exe, name, extra_remaps=None):
            return Node(
                package=pkg, executable=exe, name=name, namespace=ns,
                output='screen',
                parameters=[configured],
                remappings=common_remaps + (extra_remaps or []),
            )

        nodes += [
            lnode('nav2_controller', 'controller_server', 'controller_server'),
            lnode('nav2_planner', 'planner_server', 'planner_server'),
            lnode('nav2_behaviors', 'behavior_server', 'behavior_server'),
            lnode('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
            lnode('nav2_waypoint_follower', 'waypoint_follower',
                  'waypoint_follower'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'bond_timeout': 10.0,
                    'bond_respawn_max_duration': 10.0,
                    'node_names': LIFECYCLE_NODES,
                }],
            ),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots', default_value='2',
            description='Number of rovers (leo1..leoN) to run Nav2 for'),
        OpaqueFunction(function=launch_setup),
    ])
