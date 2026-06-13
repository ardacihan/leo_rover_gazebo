from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='Size of ArUco markers in meters'
    )

    dictionary_id_arg = DeclareLaunchArgument(
        'dictionary_id',
        default_value='0',
        description='ArUco dictionary ID (0=DICT_6X6_250)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace (e.g., leo1/)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    aruco_node = Node(
        package='leo_rover_semantic_vision',
        executable='aruco_detection_node',
        name='aruco_detection_node',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'dictionary_id': LaunchConfiguration('dictionary_id'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    return LaunchDescription([
        marker_size_arg,
        dictionary_id_arg,
        camera_topic_arg,
        robot_namespace_arg,
        use_sim_time_arg,
        aruco_node,
    ])