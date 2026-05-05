import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Launching 3 robots
    num_robots = 3

    xacro_file = os.path.join(
        get_package_share_directory('leo_rover_description'),
        'urdf',
        'leo_rover_with_sensors.urdf.xacro'
    )

    # Launch Gazebo Sim - Using default.sdf because it includes necessary sensor systems
    world = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'default.sdf'],
        output='screen'
    )

    launch_entities = [world]

    for i in range(num_robots):
        robot_ns = f"leo{i+1}"

        # 1. Process Xacro
        robot_desc = Command(['xacro ', xacro_file, ' robot_ns:=', robot_ns])

        # 2. Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_desc, value_type=str),
                'frame_prefix': f"{robot_ns}/"
            }]
        )

        # 3. Spawn Robot
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_ns,
                '-topic', f'/{robot_ns}/robot_description',
                '-x', str(i * 2.0),
                '-y', '0.0',
                '-z', '0.2'
            ],
            output='screen'
        )

        # 4. Bridge for topics
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_ns,
            arguments=[
                f'/{robot_ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{robot_ns}/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_ns}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{robot_ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        )

        launch_entities.extend([rsp_node, spawn_node, bridge_node])

    return LaunchDescription(launch_entities)