from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='leo_rover_control',
            executable='intelligence_node',
            namespace=LaunchConfiguration('robot_ns'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            # This remaps topics if they aren't exactly matching your bridge
            remappings=[
                ('scan', 'scan'),
                ('cmd_vel', 'cmd_vel'),
            ]
        )
    ])