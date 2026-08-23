import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = get_package_share_directory('leo_rover_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'amcl.launch.py')),
        launch_arguments={'map': map_yaml,
                          'use_sim_time': use_sim_time}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': 'true'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute map YAML path'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        localization, navigation,
    ])
