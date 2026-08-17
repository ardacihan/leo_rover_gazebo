"""Launch the standalone Nav2, SLAM, command guard, and collision-monitor overlay."""

from __future__ import annotations

from dataclasses import dataclass

# ROS 2's launch loader execs this file as a module it never registers in
# sys.modules. dataclasses looks the owning module up by name while building
# each class, so every @dataclass below would die on the resulting None.
# Register a proxy sharing this module's namespace before any of them run.
import sys as _sys
import types as _types

if _sys.modules.get(__name__) is None:
    _proxy = _types.ModuleType(__name__)
    _proxy.__dict__.update(globals())
    _sys.modules[__name__] = _proxy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from leo_nav2_exploration.launch_support import (
    materialize_parameter_file,
    resolve_profile_paths,
    sample_lattice_path,
)


@dataclass(frozen=True)
class ScanTopics:
    raw: str
    filtered: str


def _scan_topics(profile: str) -> ScanTopics:
    if profile == 'sim_leo1':
        return ScanTopics(raw='/leo1/scan', filtered='/leo1/scan_filtered')
    if profile == 'real_root':
        return ScanTopics(raw='/scan', filtered='/scan_filtered')
    raise ValueError(f'unsupported profile: {profile!r}')


@dataclass(frozen=True)
class CommandTopics:
    cmd_vel_nav: str
    cmd_vel_smoothed: str
    cmd_vel_guarded: str
    cmd_vel_final: str


def _command_topics(profile: str) -> CommandTopics:
    if profile == 'sim_leo1':
        prefix = '/leo1'
    elif profile == 'real_root':
        prefix = ''
    else:
        raise ValueError(f'unsupported profile: {profile!r}')
    return CommandTopics(
        cmd_vel_nav=f'{prefix}/cmd_vel_nav',
        cmd_vel_smoothed=f'{prefix}/cmd_vel_smoothed',
        cmd_vel_guarded=f'{prefix}/cmd_vel_guarded',
        cmd_vel_final=f'{prefix}/cmd_vel',
    )


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {'1', 'true', 'yes', 'on'}:
        return True
    if normalized in {'0', 'false', 'no', 'off'}:
        return False
    raise ValueError(f'expected boolean launch value, got {value!r}')


def _launch_setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    start_slam = _as_bool(LaunchConfiguration('start_slam').perform(context))
    enable_voxel = _as_bool(LaunchConfiguration('enable_voxel').perform(context))
    autostart = _as_bool(LaunchConfiguration('autostart').perform(context))
    use_respawn = _as_bool(LaunchConfiguration('use_respawn').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    navigation_start_delay = float(LaunchConfiguration('navigation_start_delay').perform(context))
    if navigation_start_delay < 0.0:
        raise ValueError('navigation_start_delay must be non-negative')

    package_share = get_package_share_directory('leo_nav2_exploration')
    smac_share = get_package_share_directory('nav2_smac_planner')
    paths = resolve_profile_paths(package_share, profile)
    lattice = sample_lattice_path(smac_share)
    topics = _command_topics(profile)
    scan_topics = _scan_topics(profile)
    use_sim_time = profile == 'sim_leo1'

    overrides = {}
    if not enable_voxel:
        overrides = {
            ('local_costmap', 'local_costmap', 'ros__parameters', 'plugins'): [
                'obstacle_layer',
                'inflation_layer',
            ],
            ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'enabled'): False,
        }

    nav2_params = materialize_parameter_file(
        paths.nav2,
        {
            '__LATTICE_FILE__': str(lattice),
            '__BT_XML__': str(paths.behavior_tree),
        },
        path_overrides=overrides,
    )

    common = {
        'output': 'screen',
        'respawn': use_respawn,
        'respawn_delay': 2.0,
        'arguments': ['--ros-args', '--log-level', log_level],
    }

    actions = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            parameters=[str(paths.scan_filter)],
            remappings=[
                ('scan', scan_topics.raw),
                ('scan_filtered', scan_topics.filtered),
            ],
            **common,
        ),
    ]

    if start_slam:
        actions.append(
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[str(paths.slam)],
                **common,
            )
        )

    # Safety path starts first. The guard publishes zero until command, scan, and odometry are fresh.
    actions.extend(
        [
            Node(
                package='leo_nav2_exploration',
                executable='velocity_guard',
                name='velocity_guard',
                parameters=[str(paths.velocity_guard)],
                **common,
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                parameters=[str(paths.collision_monitor)],
                **common,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_collision_monitor',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'bond_timeout': 10.0},
                    {'node_names': ['collision_monitor']},
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ]
    )

    nav_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[str(nav2_params)],
            remappings=[('cmd_vel', topics.cmd_vel_nav)],
            **common,
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[str(nav2_params)],
            **common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[str(nav2_params)],
            **common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[str(nav2_params)],
            remappings=[('cmd_vel', topics.cmd_vel_nav)],
            **common,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[str(nav2_params)],
            **common,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[str(nav2_params)],
            **common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[str(nav2_params)],
            remappings=[
                ('cmd_vel', topics.cmd_vel_nav),
                ('cmd_vel_smoothed', topics.cmd_vel_smoothed),
            ],
            **common,
        ),
    ]

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    nav_nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'bond_timeout': 10.0},
                {'node_names': lifecycle_nodes},
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    actions.append(TimerAction(period=navigation_start_delay, actions=nav_nodes))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='sim_leo1',
                choices=['sim_leo1', 'real_root'],
                description='Select simulator or root-level real-rover topics and frames.',
            ),
            DeclareLaunchArgument('start_slam', default_value='true'),
            DeclareLaunchArgument('enable_voxel', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('use_respawn', default_value='false'),
            DeclareLaunchArgument('navigation_start_delay', default_value='3.0'),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
