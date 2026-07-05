import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

# ── Per-map spawn coordinates: (x, y, z, R, P, Y) ──
SPAWN_POSES = {
    'husarion_office': {
        'leo1': ('0.0', '0.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('2.36', '-11.27', '0.05', '0.0', '0.0', '0.0'),
    },
    'aws_room': {
        'leo1': ('-1.76', '0.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('2.0', '2.0', '0.2', '0.0', '0.0', '0.0'),
    },
    'warehouse': {
        'leo1': ('0.0', '0.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('2.0', '2.0', '0.2', '0.0', '0.0', '0.0'),
    },
    'empty': {
        'leo1': ('0.0', '0.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('2.0', '0.0', '0.2', '0.0', '0.0', '0.0'),
    },
}


MARKER_POSES = {
    'husarion_office': [
        (0, 6.51, -11.97, 0.3, 0.0, 0.0, -0.02),
        (1, 9.39, -8.03, 0.3, 0.0, 0.0, 1.57),
        (2, 1.79, 0.0, 0.3, 0.0, 0.0, -1.57),
        (3, 9.31, 0.0, 0.3, 0.0, 0.0, -1.57),
        (4, 12.5, -3.41, 0.3, 0.0, 0.0, 3.14),
        (5, 13.33, -5.9, 0.3, 0.0, 0.0, 1.7),
        (6, 1.47, -5.42, 0.3, 0.0, 0.0, 0.0),
        (7, 4.01, -7.54, 0.3, 0.0, 0.0, 1.7),
        (8, 6.28, -4.41 , 0.3, 0.0, 0.0, 0.0),
        (9, 8.52, -7.48, 0.3, 0.0, 0.0, 1.57),
    ],

    'aws_room': [
        (0, 0.63, -5.18, 0.0, 0.0, 0.0, 1.58),
        (1, 2.97, -5.32, 0.2, 0.0, 0.0, 0.0),
        (2, -2.27, 4.66, 0.2, 0.0, 0.0, -1.57),
        (3, -0.06, 5.41, 0.38, 0.0, 0.0, -2.99),
        (4, -5.0, -1.35, 0.2, 0.0, 0.0, 0.0),
        (5, -5.0, 2.62, 0.2, 0.0, 0.0, 0.0),
        (6, 9.27, -2.01, 0.2, 0.0, 0.0, 1.62),
        (7, 5.0, 2.88, 0.2, 0.0, 0.0, 3.14),
        (8, 1.88, 0.0, 0.2, 0.0, 0.0, 1.57),
        (9, -0.37, 0.16, 0.2, 0.0, 0.0, -1.57),
    ],

    'warehouse': [
        (0, -5.0, 10.27, 0.2, 0.0, 0.0, -3.12),
        (1, -0.91, 7.6, 0.41, 0.0, 0.0, -1.57),
        (2, 2.32, 6.67, 0.24, 0.0, 0.0, -1.57),
        (3, -5.24, -9.97, 0.2, 0.0, 0.0, -0.1),
        (4, 1.15, -10.23, 0.2, 0.0, 0.0, -0.18),
        (5, 6.61, -9.76, 0.2, 0.0, 0.0, 1.57),
        (6, 6.82, -5.65, 0.41, 0.0, 0.0, -1.57),
        (7, 6.74, -3.92, 0.2, 0.0, 0.0, 3.14),
        (8, 6.66, -2.04, 0.41, 0.0, 0.0, 3.14),
        (9, -1.61, -5.8, 0.2, 0.0, 0.0, 0.0), 
    ],
}


def launch_setup(context, *args, **kwargs):
    num_robots = 2

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('leo_rover_description')
    pkg_husarion = get_package_share_directory('husarion_gz_worlds')
    pkg_leo_gazebo = get_package_share_directory('leo_rover_gazebo')

    xacro_file = os.path.join(pkg_description, 'urdf', 'leo_rover_with_sensors.urdf.xacro')

    # World paths
    husarion_sdf = os.path.join(pkg_husarion, 'worlds', 'husarion_office.sdf')
    husarion_aruco_sdf = os.path.join(pkg_husarion, 'worlds', 'husarion_office_aruco.sdf')

    if os.path.exists(husarion_aruco_sdf):
        husarion_world = husarion_aruco_sdf
    else:
        husarion_world = husarion_sdf

    worlds = {
        'husarion_office': husarion_world,
        'empty': os.path.join(pkg_husarion, 'worlds', 'empty_with_plugins.sdf'),
        'aws_room': os.path.join(pkg_leo_gazebo, 'maps', 'small_house.world'),
        'warehouse': '/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/small_warehouse.world',
    }

    world_name = LaunchConfiguration('world').perform(context)
    world_path = worlds.get(world_name, worlds['husarion_office'])

    if not os.path.exists(world_path):
        print(f"WARNING: World file {world_path} not found! Using husarion_office instead.")
        world_path = worlds['husarion_office']
        world_name = 'husarion_office'

    # ── Set up resource paths for all models ──
    aws_warehouse_path = '/ros2_ws/src/aws-robomaker-small-warehouse-world'

    resource_paths = [
        os.path.join(pkg_husarion, 'models'),
        os.path.join(pkg_leo_gazebo, 'models'),
        os.path.join(pkg_leo_gazebo, 'maps'),
        '/ros2_ws/src/leo_rover_gazebo/models',
    ]

    if os.path.exists(aws_warehouse_path):
        resource_paths.append(os.path.join(aws_warehouse_path, 'models'))
        resource_paths.append(aws_warehouse_path)

    resource_path_str = ':'.join(filter(None, resource_paths))

    # Set environment variables
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{resource_path_str}:{gazebo_model_path}" if resource_path_str else gazebo_model_path

    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_resource_path = f"{resource_path_str}:{ign_resource_path}" if resource_path_str else ign_resource_path

    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_sim_resource_path = f"{resource_path_str}:{gz_sim_resource_path}" if resource_path_str else gz_sim_resource_path

    set_gazebo_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)
    set_ign_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path)
    set_gz_sim_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_sim_resource_path)

    print(f"Loading world: {world_name} from {world_path}")
    print(f"Resource paths: {resource_path_str}")

    # ── 1. Gazebo ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # ── 2. Clock bridge ──
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    entities = [set_gazebo_path, set_ign_path, set_gz_sim_path, gz_sim, clock_bridge]

    # ── 3. Robots ──
    default_poses = SPAWN_POSES.get(world_name, SPAWN_POSES['husarion_office'])
    override = {
        'leo1': LaunchConfiguration('leo1_pose').perform(context),
        'leo2': LaunchConfiguration('leo2_pose').perform(context),
    }

    for i in range(num_robots):
        robot_ns = f'leo{i + 1}'

        if override[robot_ns]:
            spawn_x, spawn_y, spawn_z, spawn_R, spawn_P, spawn_Y = override[robot_ns].split(',')
        else:
            spawn_x, spawn_y, spawn_z, spawn_R, spawn_P, spawn_Y = default_poses[robot_ns]

        robot_desc = Command([
            'xacro', ' ', xacro_file, ' ', 'robot_ns:=', robot_ns
        ])

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
            remappings=[('/tf', '/tf'), ('tf_static', '/tf_static')],
            output='screen'
        )

        gpu_lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'gpu_lidar_tf_{robot_ns}',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                f'{robot_ns}/sensor_lidar_link',
                f'{robot_ns}/base_footprint/gpu_lidar'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        spawn = TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{robot_ns}',
                arguments=[
                    '-name', robot_ns,
                    '-topic', f'/{robot_ns}/robot_description',
                    '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                    '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y,
                ],
                output='screen'
            )]
        )

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
                f'/model/{robot_ns}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
                f'/model/{robot_ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{robot_ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                (f'/model/{robot_ns}/tf', '/tf')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        aruco_detector = Node(
            package='leo_rover_semantic_vision',
            executable='aruco_detection_node',
            namespace=robot_ns,
            name='aruco_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_ns': robot_ns
            }],
        )

        entities += [rsp, spawn, bridge, gpu_lidar_tf, aruco_detector]

    # ── 4. Spawn ArUco markers on walls ──
    for marker_data in MARKER_POSES.get(world_name, []):
        marker_id = marker_data[0]
        mx, my, mz, mroll, mpitch, myaw = marker_data[1:7]

        spawn_aruco = TimerAction(
            period=6.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_aruco_{marker_id}',
                arguments=[
                    '-name', f'aruco_{marker_id}',
                    '-x', str(mx), '-y', str(my), '-z', str(mz),
                    '-R', str(mroll), '-P', str(mpitch), '-Y', str(myaw),
                    '-string', f'''<?xml version="1.0"?>
                    <sdf version="1.9">
                      <include>
                        <uri>model://aruco_{marker_id}</uri>
                      </include>
                    </sdf>'''
                ],
                output='screen'
            )]
        )
        entities.append(spawn_aruco)

    return entities


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='husarion_office',
        description='World to launch: husarion_office, empty, aws_room, warehouse'
    )
    leo1_pose_arg = DeclareLaunchArgument(
        'leo1_pose', default_value='',
        description='Override leo1 spawn as "x,y,z,R,P,Y"'
    )
    leo2_pose_arg = DeclareLaunchArgument(
        'leo2_pose', default_value='',
        description='Override leo2 spawn as "x,y,z,R,P,Y"'
    )
    return LaunchDescription([
        world_arg, leo1_pose_arg, leo2_pose_arg,
        OpaqueFunction(function=launch_setup)
    ])