import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    package = get_package_share_directory('leo_rover_gazebo')
    default_params = os.path.join(package, 'config', 'amcl_params_leo.yaml')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server',
        name='map_server', namespace='', output='screen',
        parameters=[{'yaml_filename': map_yaml,
                     'use_sim_time': use_sim_time}],
    )
    amcl = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl',
        namespace='', output='screen', parameters=[params_file,
                                     {'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/leo1/scan')],
    )
    manager = LifecycleNode(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization', namespace='', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute map YAML path'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        map_server, amcl, manager,
    ])
