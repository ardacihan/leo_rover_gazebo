import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    num_robots = 3

    xacro_file = os.path.join(
        get_package_share_directory('leo_rover_description'),
        'urdf', 'leo_rover_with_sensors.urdf.xacro'
    )
    world_path = os.path.join(
        get_package_share_directory('leo_rover_gazebo'),
        'worlds', 'leo_world.sdf'
    )

    # Humble uses 'ign gazebo' instead of 'gz sim'
    gz_sim = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path], output='screen')

    # Change gz.msgs.Clock to ignition.msgs.Clock
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    launch_entities = [gz_sim, clock_bridge]

    for i in range(num_robots):
        robot_ns = f"leo{i + 1}"
        robot_desc = Command(['xacro', ' ', xacro_file, ' ', 'robot_ns:=', robot_ns])

        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot_ns,
            parameters=[{
                'robot_description': ParameterValue(robot_desc, value_type=str),
                'use_sim_time': True
            }],
            output='screen'
        )

        # Humble uses ros_ign_sim instead of ros_gz_sim
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_ns, '-topic', f'/{robot_ns}/robot_description',
                       '-x', str(i * 2.0), '-y', '0.0', '-z', '0.2'],
            output='screen'
        )

        # Change gz.msgs to ignition.msgs for all bridge arguments
        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            namespace=robot_ns,
            arguments=[
                f'/{robot_ns}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                f'/{robot_ns}/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                f'/{robot_ns}/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                f'/{robot_ns}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                f'/{robot_ns}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                f'/{robot_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                f'/{robot_ns}/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        launch_entities += [rsp, spawn, bridge]

    return LaunchDescription(launch_entities)