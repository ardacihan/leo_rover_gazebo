import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    num_robots = 1

    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')
    pkg_description  = get_package_share_directory('leo_rover_description')
    pkg_gazebo       = get_package_share_directory('leo_rover_gazebo')

    xacro_file  = os.path.join(pkg_description, 'urdf', 'leo_rover_with_sensors.urdf.xacro')
    world_path  = os.path.join(pkg_gazebo,      'worlds', 'leo_world.sdf')

    # ── 1. Gazebo — use ros_gz_sim's launcher so GZ_SIM_* env is set correctly
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # ── 2. Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    entities = [gz_sim, clock_bridge]

    for i in range(num_robots):
        robot_ns = f'leo{i + 1}'
        spawn_x  = str(float(i) * 2.5)

        robot_desc = Command([
            'xacro', ' ', xacro_file, ' ', 'robot_ns:=', robot_ns
        ])

        # ── robot_state_publisher (one per robot, in its namespace)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_ns,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(robot_desc, value_type=str),
                'use_sim_time': True,
                'publish_frequency': 50.0,
            }],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
            output='screen'
        )

        # ── Spawn: triggered 5 s after gz_sim starts to ensure world is ready.
        # ros_gz_sim's gz_sim.launch.py names its Gazebo process 'gzserver'
        # on some versions; a fixed delay is the most reliable approach here.
        spawn = TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{robot_ns}',
                arguments=[
                    '-name',  robot_ns,
                    '-topic', f'/{robot_ns}/robot_description',
                    '-x', spawn_x, '-y', '0.0', '-z', '0.2',
                ],
                output='screen'
            )]
        )

        # ── Per-robot bridge
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_ns,
            name=f'bridge_{robot_ns}',
            arguments=[
                f'/{robot_ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{robot_ns}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{robot_ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{robot_ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{robot_ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{robot_ns}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'/model/{robot_ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                (f'/model/{robot_ns}/tf', '/tf'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        entities += [rsp, spawn, bridge]

    return LaunchDescription(entities)