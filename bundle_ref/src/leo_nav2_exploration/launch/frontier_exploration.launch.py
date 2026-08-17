"""Launch the pinned frontier explorer in cold-idle mode by default."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from leo_nav2_exploration.launch_support import materialize_parameter_file, resolve_profile_paths


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    autostart = _as_bool(LaunchConfiguration('autostart').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    package_share = get_package_share_directory('leo_nav2_exploration')
    paths = resolve_profile_paths(package_share, profile)
    params = materialize_parameter_file(
        paths.frontier,
        {},
        path_overrides={
            ('frontier_explorer', 'ros__parameters', 'autostart'): autostart,
            ('frontier_explorer', 'ros__parameters', 'control_service_enabled'): True,
        },
    )
    return [
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',
            parameters=[str(params)],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='sim_leo1',
                choices=['sim_leo1', 'real_root'],
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='false',
                description='Keep false until manual NavigateToPose doorway tests pass.',
            ),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
