"""Nav2 + SLAM on a localhost ROS domain so costmaps never reach the rover SBC.

Run the firmware bridge first (rover domain 2, UDP-only FastDDS). Then start
RealSense on domain 22 with ROS_LOCALHOST_ONLY=1, then:

    ROS_DOMAIN_ID=22 ROS_LOCALHOST_ONLY=1 \\
      ros2 launch leo_nav2_exploration isolated_real_navigation.launch.py

Lidar and the RealSense must also run on domain 22. Do not start this on
domain 2, and do not set FASTRTPS_DEFAULT_PROFILES_FILE here: that would
replace Humble's localhost-only FastDDS profile and leak costmaps onto
Ethernet.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('leo_nav2_exploration')
    # Leo URDF: base_footprint -> base_link is 0.19783 m up. Camera TF is
    # published from base_link; without this link the depth cloud is unusable
    # on the isolated domain (bringup's copy lives on domain 2).
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.19783',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
        ],
        output='screen',
    )
    # The C1 is mounted yaw-π. A yaw-0 TF makes scan angle 0 look like the
    # robot's front while that ray actually points out the back, so Nav2
    # drives into the wall it thinks is behind it (jetson-02 2026-08-25).
    # Offsets are the Leo mount measured on rover 4 (2026-08-13).
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_laser',
        arguments=[
            '--x', '0.0775', '--y', '0.04', '--z', '0.2458',
            '--yaw', '3.14159',
            '--frame-id', 'base_footprint', '--child-frame-id', 'laser',
        ],
        output='screen',
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'real_navigation.launch.py')
        ),
        launch_arguments={
            'enable_voxel': 'true',
            'publish_camera_tf': 'true',
            'cloud_input_topic': '/camera/camera/depth/color/points',
            'start_slam': 'true',
            'autostart': 'true',
            'navigation_start_delay': '8.0',
        }.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '22'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        base_link_tf,
        laser_tf,
        navigation,
    ])
