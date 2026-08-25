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

Two-rover use (night 2026-08-25): pass `robot_ns:=rob_a`. Every topic in the
chain and every frame gains the prefix (`/rob_a/scan` -> ... ->
`/rob_a/cmd_vel`, frames `rob_a/map`, `rob_a/odom`, `rob_a/base_footprint`),
slam_toolbox keeps its load-bearing absolute-`/map` remap (now to
`/rob_a/map`), and `/tf`, `/tf_static` stay global. **The default (empty)
leaves single-rover behaviour byte-for-byte what the 2026-08-20 field runs
used** -- the node list below is only assembled differently when the argument
is non-empty.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# tf2 subscribes to *relative* tf topics, so a namespaced node silently reads
# /rob_a/tf and loses map->base unless the topics are pinned back to global.
TF_GLOBAL = [('tf', '/tf'), ('tf_static', '/tf_static'),
             ('/tf', '/tf'), ('/tf_static', '/tf_static')]


def _yaml_params(path, node_key):
    """One node's ros__parameters dict from a config file.

    A parameter file whose top-level key is a bare node name does not match
    that node under a namespace (Humble matches the full name), so in
    namespaced mode the yaml silently fails to load: the sim rehearsal
    caught collision_monitor aborting on 'StopZone.type is not initialized'
    and slam running on defaults. Inline dicts apply unconditionally, so
    under a namespace the file is loaded here and passed inline. The
    default (un-namespaced) path keeps passing the file itself,
    byte-for-byte as fielded.
    """
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return dict(data.get(node_key, {}).get('ros__parameters', {}))


def _launch_setup(context):
    share = get_package_share_directory('leo_nav2_exploration')
    cfg = os.path.join(share, 'config', 'real')

    ns = LaunchConfiguration('robot_ns').perform(context).strip('/')
    p = f'/{ns}' if ns else ''          # topic prefix
    fp = f'{ns}/' if ns else ''         # frame prefix
    nskw = {'namespace': ns} if ns else {}
    tf = TF_GLOBAL if ns else []
    # Sim rehearsal only. The real configs pin use_sim_time false; under the
    # sim clock every stamp would be wrong without this override.
    sim_time = LaunchConfiguration(
        'use_sim_time').perform(context).lower() == 'true'
    st = [{'use_sim_time': True}] if sim_time else []

    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    if ns and scan_topic == '/scan':
        # Untouched default under a namespace: the rover's drivers publish
        # under the same prefix.
        scan_topic = f'{p}/scan'

    common = {'output': 'screen', 'respawn': False}

    def cfg_params(fname, node_key):
        """File path in the default mode; inline dict under a namespace."""
        path = os.path.join(cfg, fname)
        return _yaml_params(path, node_key) if ns else path

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[cfg_params('scan_filter.yaml', 'scan_to_scan_filter_chain')] + st,
        remappings=[('scan', scan_topic),
                    ('scan_filtered', f'{p}/scan_filtered')] + tf,
        **nskw, **common,
    )

    # slam.yaml consumes /scan_uniform, which the navigation overlay's
    # scan_normalizer produces in the single-rover stack. Under a robot_ns
    # this launch must be self-contained, so the normalizer comes up here
    # (namespaced mode only -- the default node list is untouched).
    normalizer = Node(
        package='leo_nav2_exploration',
        executable='scan_normalizer',
        name='scan_normalizer',
        parameters=[{'input_topic': f'{p}/scan_filtered',
                     'output_topic': f'{p}/scan_uniform'}] + st,
        remappings=tf,
        **nskw, **common,
    ) if ns else None

    slam_params = [cfg_params('slam.yaml', 'slam_toolbox')]
    if ns:
        slam_params.append({
            'odom_frame': f'{fp}odom',
            'map_frame': f'{fp}map',
            'base_frame': f'{fp}base_footprint',
            'scan_topic': f'{p}/scan_uniform',
        })
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=slam_params + st,
        # The load-bearing remap: slam_toolbox publishes an ABSOLUTE /map,
        # so two rovers on one domain clobber each other without it. Same
        # pattern as slam_multi.launch.py in sim.
        remappings=([('/map', f'{p}/map'),
                     ('/map_metadata', f'{p}/map_metadata')] + tf) if ns else [],
        condition=IfCondition(LaunchConfiguration('start_slam')),
        **nskw, **common,
    )

    # The smoother is worth keeping even under teleop: it bounds acceleration,
    # and a skid-steer that is stepped from 0 to full command slips its wheels,
    # which is precisely the error the odometry cannot measure.
    smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[cfg_params('nav2.yaml', 'velocity_smoother')] + st,
        remappings=[('cmd_vel', f'{p}/cmd_vel_nav'),
                    ('cmd_vel_smoothed', f'{p}/cmd_vel_smoothed')] + tf,
        **nskw, **common,
    )

    guard_odom = LaunchConfiguration('guard_odom_topic').perform(context)
    guard_params = [cfg_params('velocity_guard.yaml', 'velocity_guard')]
    if ns:
        guard_params.append({
            'input_topic': f'{p}/cmd_vel_smoothed',
            'output_topic': f'{p}/cmd_vel_guarded',
            'scan_topic': f'{p}/scan_filtered',
            # The real rover publishes /{ns}/wheel_odom; the sim rehearsal
            # points this at /{ns}/odom_wheel_like instead.
            'odom_topic': guard_odom or f'{p}/wheel_odom',
            'battery_topic': f'{p}/battery_state',
        })
    elif guard_odom:
        guard_params.append({'odom_topic': guard_odom})
    guard = Node(
        package='leo_nav2_exploration',
        executable='velocity_guard',
        name='velocity_guard',
        parameters=guard_params + st,
        remappings=tf,
        **nskw, **common,
    )

    cm_params = [
        cfg_params('collision_monitor.yaml', 'collision_monitor'),
        # FootprintApproach subscribes to /local_costmap/published_footprint,
        # which only Nav2's local costmap publishes. With no Nav2 running it
        # would sit permanently unarmed while looking configured, so it is
        # dropped here and the fixed StopZone and SlowdownZone do the work.
        {'polygons': ['StopZone', 'SlowdownZone']},
    ]
    if ns:
        cm_params.append({
            'base_frame_id': f'{fp}base_footprint',
            'odom_frame_id': f'{fp}odom',
            'cmd_vel_in_topic': f'{p}/cmd_vel_guarded',
            'cmd_vel_out_topic': f'{p}/cmd_vel',
            'scan': {'topic': f'{p}/scan_filtered'},
        })
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=cm_params + st,
        remappings=tf,
        **nskw, **common,
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        parameters=[{'use_sim_time': sim_time},
                    {'autostart': True},
                    {'bond_timeout': 10.0},
                    # velocity_smoother is a *lifecycle* node. Started without
                    # a manager it sits in UNCONFIGURED, publishes nothing, and
                    # the whole command chain is silently dead from the top --
                    # the rover simply never moves and no node reports an error.
                    {'node_names': ['collision_monitor', 'velocity_smoother']}],
        output='screen',
        **nskw,
    )

    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    if ns and use_ekf:
        # odometry_fusion.launch.py is not namespaced; under a robot_ns the
        # EKF belongs to the rover's own bringup (real_bringup.launch.py
        # robot_ns:=...). Failing loudly beats two EKFs fighting over
        # odom -> base_footprint.
        raise RuntimeError(
            'use_ekf:=true is not supported together with robot_ns; '
            'run the EKF from leo_rover_real_bringup instead')
    odometry_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'odometry_fusion.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    aruco_args = {
        'profile': 'real',
        'use_sim_time': 'false',
        'marker_length': LaunchConfiguration('marker_length'),
        'allowed_ids': LaunchConfiguration('allowed_ids'),
        'dictionary': LaunchConfiguration('dictionary'),
    }
    if ns:
        aruco_args['robot_ns'] = ns
        aruco_args['detection_topic'] = f'{p}/tag_detections'
    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'aruco.launch.py')),
        launch_arguments=aruco_args.items(),
        condition=IfCondition(LaunchConfiguration('use_aruco')),
    )

    nodes = [scan_filter, slam, smoother, guard, collision_monitor,
             lifecycle, odometry_fusion, aruco]
    if normalizer is not None:
        nodes.insert(1, normalizer)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Namespace of the rover this stack drives (rob_a / rob_b). Default
        # empty = the exact single-rover field configuration of 2026-08-20.
        DeclareLaunchArgument('robot_ns', default_value=''),
        # Sim rehearsal knobs; both inert at their defaults.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'guard_odom_topic', default_value='',
            description='velocity_guard odom source; empty = /{ns}/wheel_odom'),
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
        OpaqueFunction(function=_launch_setup),
    ])
