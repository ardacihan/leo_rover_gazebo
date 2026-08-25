"""Firmware-only bringup on jetson-02 (ROS_DOMAIN_ID=2, UDP-only FastDDS).

Stock leo_bringup also starts web_video_server, rosbridge, rosapi, and the
CSI camera on the firmware domain. Those flood the firmware endpoint; kill
them after this launch. Mapping camera is the RealSense on domain 22.

Do not use this on jetson-04 / domain 4.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    overlay = get_package_share_directory('leo_nav2_exploration')
    bringup = os.path.join(
        get_package_share_directory('leo_bringup'),
        'launch',
        'leo_bringup.launch.xml',
    )
    udp_xml = os.path.join(overlay, 'config', 'real', 'fastdds_udp_only.xml')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '2'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', udp_xml),
        IncludeLaunchDescription(FrontendLaunchDescriptionSource(bringup)),
    ])
