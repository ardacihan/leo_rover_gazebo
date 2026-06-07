from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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