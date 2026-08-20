"""Teleoperated mapping on the physical rover: SLAM and safety, no autonomy.

    ros2 launch leo_nav2_exploration real_mapping.launch.py

This is the stack with the fewest ways to fail. `real_navigation.launch.py`
adds a planner, a controller, a behaviour tree and a frontier explorer, and
every one of them can decide the rover should not move -- which in simulation
cost between 35% and 60% of the run in stalls. Here the operator is the
planner, and the parts that build the map are exactly the parts that run.

What comes up:

    laser_filters       /scan -> /scan_filtered, masking the camera bracket's
                        own return, because SLAM must never see it
    slam_toolbox        async mapping against /scan_filtered
    velocity_guard      zeroes the command if scan, odometry or the command
                        itself goes stale
    collision_monitor   sole publisher of /cmd_vel; stop and slowdown zones
    EKF (optional)      wheel odometry + gyro -> odom -> base_footprint
    ArUco (optional)    marker detection off the RealSense colour stream

Teleop enters the *top* of the safety chain, not the bottom:

    teleop -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
           -> velocity_guard -> /cmd_vel_guarded -> collision_monitor -> /cmd_vel

so a joystick command is subject to the same stop zones as an autonomous one.
Point the teleop node at `/cmd_vel_nav`; anything published straight to
`/cmd_vel` bypasses every protection here and fights the collision monitor for
the topic.

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/cmd_vel_nav

Save the map when done:

    ros2 run nav2_map_server map_saver_cli -f ~/office_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('leo_nav2_exploration')
    cfg = os.path.join(share, 'config', 'real')

    common = {'output': 'screen', 'respawn': False}

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[os.path.join(cfg, 'scan_filter.yaml')],
        remappings=[('scan', LaunchConfiguration('scan_topic')),
                    ('scan_filtered', '/scan_filtered')],
        **common,
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(cfg, 'slam.yaml')],
        condition=IfCondition(LaunchConfiguration('start_slam')),
        **common,
    )

    # The smoother is worth keeping even under teleop: it bounds acceleration,
    # and a skid-steer that is stepped from 0 to full command slips its wheels,
    # which is precisely the error the odometry cannot measure.
    smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[os.path.join(cfg, 'nav2.yaml')],
        remappings=[('cmd_vel', '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel_smoothed')],
        **common,
    )

    guard = Node(
        package='leo_nav2_exploration',
        executable='velocity_guard',
        name='velocity_guard',
        parameters=[os.path.join(cfg, 'velocity_guard.yaml')],
        **common,
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[
            os.path.join(cfg, 'collision_monitor.yaml'),
            # FootprintApproach subscribes to /local_costmap/published_footprint,
            # which only Nav2's local costmap publishes. With no Nav2 running it
            # would sit permanently unarmed while looking configured, so it is
            # dropped here and the fixed StopZone and SlowdownZone do the work.
            {'polygons': ['StopZone', 'SlowdownZone']},
        ],
        **common,
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'bond_timeout': 10.0},
                    # velocity_smoother is a *lifecycle* node. Started without
                    # a manager it sits in UNCONFIGURED, publishes nothing, and
                    # the whole command chain is silently dead from the top --
                    # the rover simply never moves and no node reports an error.
                    {'node_names': ['collision_monitor', 'velocity_smoother']}],
        output='screen',
    )

    odometry_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'odometry_fusion.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'aruco.launch.py')),
        launch_arguments={
            'profile': 'real',
            'use_sim_time': 'false',
            'marker_length': LaunchConfiguration('marker_length'),
            'allowed_ids': LaunchConfiguration('allowed_ids'),
            'dictionary': LaunchConfiguration('dictionary'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_aruco')),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        # Off by default: the EKF takes ownership of odom -> base_footprint and
        # the rover's own bringup must stop publishing it first. See
        # config/real/ekf.yaml.
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_aruco', default_value='false'),
        # Side of the printed black square, measured with a ruler. Not the
        # sheet, not the white border.
        DeclareLaunchArgument('marker_length', default_value='0.15'),
        # Comma-separated ids you physically placed; anything else is rejected.
        DeclareLaunchArgument('allowed_ids', default_value='1,2,3,4,5,6,7,8'),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50'),
        scan_filter,
        slam,
        smoother,
        guard,
        collision_monitor,
        lifecycle,
        odometry_fusion,
        aruco,
    ])
