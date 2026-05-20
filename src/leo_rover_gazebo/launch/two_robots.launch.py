import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    num_robots = 2

    xacro_file = os.path.join(
        get_package_share_directory('leo_rover_description'),
        'urdf', 'leo_rover_with_sensors.urdf.xacro'
    )
    world_path = os.path.join(
        get_package_share_directory('leo_rover_gazebo'),
        'worlds', 'leo_world.sdf'
    )

    gz_sim = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_path], output='screen')

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
                'use_sim_time': True,
                'frame_prefix': f"{robot_ns}/"
            }],
            # Force static transforms (lidar, camera, wheels) to the global TF topics
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ],
            output='screen'
        )

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_ns, '-topic', f'/{robot_ns}/robot_description',
                       '-x', str(i * 2.0), '-y', '0.0', '-z', '0.2'],
            output='screen'
        )

        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            namespace=robot_ns,
            arguments=[
                f'/{robot_ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{robot_ns}/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'/{robot_ns}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{robot_ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_ns}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{robot_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{robot_ns}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

                # Odometry and Velocity
                f'/{robot_ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{robot_ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # TF
                f'/{robot_ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            # Force Gazebo's odometry transforms to the global TF topic
            remappings=[
                (f'/{robot_ns}/tf', '/tf')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        launch_entities += [rsp, spawn, bridge]

    return LaunchDescription(launch_entities)