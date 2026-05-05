import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

NUM_ROBOTS = 3

def generate_launch_description():


    xacro_file = os.path.join(
        get_package_share_directory('leo_rover_description'),
        'urdf',
        'leo_rover_with_sensors.urdf.xacro'
    )

    # Launch Gazebo
    world = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    launch_entities = [world]

    for i in range(NUM_ROBOTS):
        robot_ns = f"leo{i+1}"

        # 1. Native ROS 2 way to parse Xacro dynamically
        robot_desc = Command(['xacro ', xacro_file, ' robot_ns:=', robot_ns])

        # 2. Robot State Publisher (Mandatory for TF, RViz, and ros2_control)
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'frame_prefix': f"{robot_ns}/"
            }]
        )

        # 3. Spawn in Gazebo (Now reads directly from the RSP topic)
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_ns,
                '-topic', f'/{robot_ns}/robot_description',
                '-x', str(i * 2.0),
                '-z', '0.1'
            ],
            output='screen'
        )

        launch_entities.extend([rsp_node, spawn_node])

    return LaunchDescription(launch_entities)