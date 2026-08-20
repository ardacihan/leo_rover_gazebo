"""Shadow-replay of the real-rover stack against a drive bag.

Launches the exact node set the rover runs (scan filter, slam_toolbox, the
full Nav2 overlay, velocity guard, collision monitor, explore_lite) with two
replay-only changes:

  * every node runs on the bag's clock (use_sim_time: true, bag played with
    --clock) -- the recorded 2026-08-20 stamps are the only time that exists;
  * the safety chain shadows the human driver instead of commanding a robot:
    the bag's driven /cmd_vel feeds velocity_guard -> collision_monitor, whose
    verdict is published as /cmd_vel_shadow. Divergence between /cmd_vel and
    /cmd_vel_shadow is exactly where the stack would have intervened.

Nav2's own controller output stays on /cmd_vel_nav, unconsumed: goals, plans
and costmaps are real, motion is the bag's.

    ros2 launch <this file> config_dir:=/tmp/drive_replay_cfg
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _patch(src, dst, replacements):
    text = open(src, encoding='utf-8').read()
    text = text.replace('use_sim_time: false', 'use_sim_time: true')
    for old, new in replacements.items():
        text = text.replace(old, new)
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return dst


def _launch_setup(context):
    share = get_package_share_directory('leo_nav2_exploration')
    # 'real' = current tuning; 'real_baseline_2026-08-20' = frozen snapshot.
    cfg = os.path.join(share, 'config',
                       LaunchConfiguration('config_profile').perform(context))
    out = LaunchConfiguration('config_dir').perform(context)
    os.makedirs(out, exist_ok=True)

    bt_xml = os.path.join(share, 'behavior_trees',
                          'navigate_to_pose_doorway_recovery.xml')
    p = {}
    p['scan_filter'] = _patch(os.path.join(cfg, 'scan_filter.yaml'),
                              os.path.join(out, 'scan_filter.yaml'), {})
    p['slam'] = _patch(os.path.join(cfg, 'slam.yaml'),
                       os.path.join(out, 'slam.yaml'), {})
    nav2_repl = {'__BT_XML__': bt_xml}
    # The configs name the rover-4 camera topic; in replay the depth bridge
    # publishes the cloud under the RealSense default namespace instead.
    nav2_repl['topic: /rob_4/camera/depth/color/points'] = \
        'topic: /camera/camera/depth/color/points'
    lidar_only = LaunchConfiguration('lidar_only').perform(context).lower() \
        in ('1', 'true', 'yes', 'on')
    if lidar_only:
        # Drop the camera layer from the plugin list entirely -- the local
        # costmap then sees obstacles through the lidar alone, mirroring
        # `enable_voxel:=false` on the rover.
        nav2_repl['- camera_obstacle_layer'] = '# camera layer dropped (lidar_only)'
    p['nav2'] = _patch(os.path.join(cfg, 'nav2.yaml'),
                       os.path.join(out, 'nav2.yaml'), nav2_repl)
    p['guard'] = _patch(os.path.join(cfg, 'velocity_guard.yaml'),
                        os.path.join(out, 'velocity_guard.yaml'),
                        {'input_topic: /cmd_vel_smoothed':
                         'input_topic: /cmd_vel'})
    p['monitor'] = _patch(os.path.join(cfg, 'collision_monitor.yaml'),
                          os.path.join(out, 'collision_monitor.yaml'),
                          {'cmd_vel_out_topic: /cmd_vel':
                           'cmd_vel_out_topic: /cmd_vel_shadow'})
    # Shadow mode never moves the robot toward a goal, so the real 60 s
    # progress timeout would blacklist every frontier early in the run and the
    # explorer would fall silent. Effectively disable it: we want to see the
    # goal the stack would pick at every moment of the drive.
    p['explore'] = _patch(os.path.join(cfg, 'explore.yaml'),
                          os.path.join(out, 'explore.yaml'),
                          {'progress_timeout: 60.0':
                           'progress_timeout: 100000.0'})

    common = {'output': 'screen',
              'arguments': ['--ros-args', '--log-level', 'info']}

    actions = [
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain', parameters=[p['scan_filter']],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
             **common),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[p['slam']], **common),
        Node(package='leo_nav2_exploration', executable='velocity_guard',
             name='velocity_guard', parameters=[p['guard']], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', parameters=[p['monitor']], **common),
        # Managers stay on the wall clock with bonds disabled: the bag clock
        # only starts ticking when play begins, and a sim-time manager would
        # hang in its own activation timeouts until then.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision_monitor',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'bond_timeout': 0.0,
                          'node_names': ['collision_monitor']}],
             output='screen'),
    ]

    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=[p['nav2']],
             remappings=[('cmd_vel', '/cmd_vel_nav')], **common),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', parameters=[p['nav2']], **common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=[p['nav2']], **common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=[p['nav2']],
             remappings=[('cmd_vel', '/cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=[p['nav2']], **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', parameters=[p['nav2']], **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[p['nav2']],
             remappings=[('cmd_vel', '/cmd_vel_nav'),
                         ('cmd_vel_smoothed', '/cmd_vel_smoothed')],
             **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'bond_timeout': 0.0,
                          'node_names': ['controller_server', 'smoother_server',
                                         'planner_server', 'behavior_server',
                                         'bt_navigator', 'waypoint_follower',
                                         'velocity_smoother']}],
             output='screen'),
    ]
    actions.append(TimerAction(period=3.0, actions=nav_nodes))

    explorer = Node(package='explore_lite', executable='explore',
                    name='explore_node', parameters=[p['explore']], **common)
    actions.append(TimerAction(period=12.0, actions=[explorer]))

    # Same filter the rover runs: raw bag-derived cloud in, filtered cloud
    # out. Harmless under the baseline profile, whose costmap reads the raw
    # topic directly.
    actions.append(
        Node(package='leo_nav2_exploration', executable='cloud_filter',
             name='cloud_filter',
             parameters=[{'use_sim_time': True}], **common))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir',
                              default_value='/tmp/drive_replay_cfg'),
        DeclareLaunchArgument('config_profile', default_value='real'),
        DeclareLaunchArgument('lidar_only', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
