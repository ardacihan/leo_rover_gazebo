from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_params = {
        'use_sim_time': True,

        'scan_topic': '/leo1/scan',

        'odom_frame': 'leo1/odom',
        'base_frame': 'leo1/base_link',
        'map_frame': 'map',

        'mode': 'mapping',

        'resolution': 0.05,
        'max_laser_range': 20.0,

        'transform_timeout': 0.5,
        'tf_buffer_duration': 30.0,

        'minimum_travel_distance': 0.1,
        'minimum_travel_heading': 0.1,

        'map_update_interval': 1.0,

        'enable_interactive_mode': True,
    }

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/ros2_ws/src/leo_rover_gazebo/config/slam_params_leo.yaml'],
            remappings=[('/scan', '/leo1/scan')]
        )
    ])