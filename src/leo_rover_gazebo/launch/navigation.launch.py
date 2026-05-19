import os
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    leo_nav_share = get_package_share_directory('leo_nav')

    navigation = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(leo_nav_share, 'launch', 'navigation.launch.xml')
        ),
        launch_arguments={
            'scan_topic': '/leo1/scan',
            'use_stereo_camera': 'false',
        }.items()
    )

    namespaced = GroupAction([
        PushRosNamespace('leo1'),
        navigation
    ])

    return LaunchDescription([namespaced])