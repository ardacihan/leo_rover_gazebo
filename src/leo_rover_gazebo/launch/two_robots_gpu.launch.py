import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def resolve_world(world):
    """Accept a full path or a bare world name (searched in known world dirs)."""
    if os.path.sep in world or world.endswith('.sdf'):
        return world
    ws_root = os.environ.get('ROS2_WS', '/ros2_ws')
    candidates = [
        os.path.join(ws_root, 'src', 'husarion_gz_worlds', 'worlds', f'{world}.sdf'),
        os.path.join(get_package_share_directory('leo_rover_gazebo'),
                     'worlds', f'{world}.sdf'),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise FileNotFoundError(f'World "{world}" not found in: {candidates}')


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    # Per-robot spawn poses (x, y). leo1 sits at the world origin (the
    # single-robot spawn, verified free in every authored world); leo2 is
    # offset alongside it. These offsets are the ground truth handed to
    # multirobot_map_merge as the robots' known initial poses, so keep them
    # in sync with map_merge_leo.launch.py.
    default_spawns = [(0.0, 0.0), (1.5, 0.0), (0.0, 1.5), (1.5, 1.5)]

    pkg_description = get_package_share_directory('leo_rover_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'leo_rover_with_sensors.urdf.xacro')
    ws_root = os.environ.get('ROS2_WS', '/ros2_ws')
    world_path = resolve_world(LaunchConfiguration('world').perform(context))

    # Headless server: ign gazebo v6 matches ros_gz_bridge/create transport.
    # Calling ros_gz_sim/gz_sim.launch.py scans every package.xml to collect
    # Gazebo export paths. On a Windows-mounted workspace that scan can take
    # indefinitely before Gazebo is even spawned. Resource paths are already
    # supplied by the entrypoint and sim script, so launch the exact server
    # command directly.
    # WSL GPU libs are enabled only when the image has the patched Ogre build
    # (see docker/ogre-wsl-gpu.patch). Otherwise Mesa software GL is used.
    patched_ogre = os.path.join(ws_root, 'docker', 'patched', 'RenderSystem_GL3Plus.so')
    use_wsl_gpu = (
        os.path.exists('/usr/local/share/leo_rover_gazebo/ogre_wsl_gpu_patched')
        or os.path.exists(patched_ogre)
    )
    gpu_env = {'LIBGL_ALWAYS_INDIRECT': '0'}
    if use_wsl_gpu:
        gpu_env['LD_LIBRARY_PATH'] = '/usr/lib/wsl/lib'

    # The HEADLESS SERVER renders every gpu_lidar / camera sensor. Without the
    # WSL GPU libs on its LD_LIBRARY_PATH it falls back to software (Mesa) GL on
    # the CPU - the GPU sits idle and the sim crawls at ~1x real-time. Giving it
    # gpu_env moves sensor rendering onto the GPU. --render-engine ogre2 selects
    # the accelerated backend.
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_path,
             '--force-version', '6'],
        output='screen',
        additional_env=gpu_env,
    )

    # GUI must match the ign gazebo v6 server or the 3D view stays black.
    gui_env = gpu_env

    gz_gui = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '--force-version', '6'],
            output='screen',
            additional_env=gui_env,
            condition=IfCondition(LaunchConfiguration('gui')),
        )]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    entities = [gz_server, gz_gui, clock_bridge]

    for i in range(num_robots):
        robot_ns = f'leo{i + 1}'
        sx, sy = default_spawns[i] if i < len(default_spawns) \
            else (float(i) * 1.5, 0.0)
        spawn_x, spawn_y = str(sx), str(sy)

        robot_desc = Command([
            'xacro', ' ', xacro_file, ' ', 'robot_ns:=', robot_ns,
            ' ', 'enable_camera:=', LaunchConfiguration('enable_camera')
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
            period=8.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{robot_ns}',
                arguments=[
                    '-name', robot_ns,
                    '-topic', f'/{robot_ns}/robot_description',
                    '-x', spawn_x, '-y', spawn_y, '-z', '0.2',
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
            ],
            remappings=[
                (f'/model/{robot_ns}/tf', '/tf')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        entities += [rsp, spawn, bridge, gpu_lidar_tf]

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='husarion_office',
            description='World name (husarion_office, husarion_world, '
                        'leo_world, ...) or full path to an .sdf file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo GUI client'),
        DeclareLaunchArgument(
            'num_robots', default_value='1',
            description='Number of Leo rovers to spawn (leo1..leoN)'),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Spawn the RGBD camera sensor (disable for fast '
                        'lidar-only exploration)'),
        OpaqueFunction(function=launch_setup),
    ])
