import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from marker_spawn import marker_spawn_actions  # noqa: E402
from spawn_poses import FALLBACK_SPAWNS, SPAWN_POSES  # noqa: E402


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
    # The Gazebo OdometryPublisher reads the *true* model pose, so bridging its
    # TF onto /tf hands SLAM a perfect odom->base_link prior that no physical
    # rover can supply. Set gt_odom_tf:=false to divert it and let
    # scripts/sim_realism_odom.py own that transform instead.
    gt_odom_tf = LaunchConfiguration('gt_odom_tf').perform(context).lower() \
        in ('true', '1', 'yes')

    pkg_description = get_package_share_directory('leo_rover_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'leo_rover_with_sensors.urdf.xacro')
    ws_root = os.environ.get('ROS2_WS', '/ros2_ws')
    world_name = LaunchConfiguration('world').perform(context)
    world_path = resolve_world(world_name)

    # Per-robot spawn poses: explicit leo{i}_pose argument wins, then the
    # per-room table for this world, then the legacy side-by-side fallback.
    world_key = os.path.basename(world_name).replace('.sdf', '')
    authored = SPAWN_POSES.get(world_key, {})
    overrides = {
        f'leo{i + 1}': LaunchConfiguration(f'leo{i + 1}_pose').perform(context)
        for i in range(min(num_robots, 2))
    }

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
        if overrides.get(robot_ns):
            pose = overrides[robot_ns].split(',')
        elif robot_ns in authored:
            pose = list(authored[robot_ns])
        else:
            sx, sy = FALLBACK_SPAWNS[i] if i < len(FALLBACK_SPAWNS) \
                else (float(i) * 1.5, 0.0)
            pose = [str(sx), str(sy), '0.2', '0.0', '0.0', '0.0']
        if len(pose) != 6:
            raise ValueError(
                f'{robot_ns}_pose must be "x,y,z,R,P,Y"; got {pose!r}')
        spawn_x, spawn_y, spawn_z, spawn_R, spawn_P, spawn_Y = \
            (p.strip() for p in pose)

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
            ],
            remappings=[
                (f'/model/{robot_ns}/tf',
                 '/tf' if gt_odom_tf else f'/{robot_ns}/tf_ground_truth')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        entities += [rsp, spawn, bridge, gpu_lidar_tf]

    # Wall-mounted ArUco markers. office_world and depot_world carry textured
    # markers in the .sdf already; husarion_office has none, so they are
    # spawned here from the same ground-truth yaml the scoring reads. 'auto'
    # spawns only when the world file has no ArUco texture of its own, so a
    # world is never given two overlapping sets. Skipped with the cameras off,
    # since nothing can then see them.
    spawn_markers = LaunchConfiguration('spawn_markers').perform(context).lower()
    cameras_on = LaunchConfiguration('enable_camera').perform(context).lower() \
        in ('true', '1', 'yes')
    if spawn_markers == 'auto':
        try:
            with open(world_path) as fh:
                want_markers = 'aruco_markers/textures' not in fh.read()
        except OSError:
            want_markers = True
    else:
        want_markers = spawn_markers in ('true', '1', 'yes')
    if want_markers and cameras_on:
        actions = marker_spawn_actions(world_key, period=14.0)
        print(f'[two_robots_gpu] spawning {len(actions)} ArUco markers '
              f'for {world_key}')
        entities += actions
    elif cameras_on:
        print(f'[two_robots_gpu] using the ArUco markers baked into {world_key}')

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
        DeclareLaunchArgument(
            'leo1_pose', default_value='',
            description='Override leo1 spawn as "x,y,z,R,P,Y" (empty = the '
                        'per-room pose authored for this world)'),
        DeclareLaunchArgument(
            'leo2_pose', default_value='',
            description='Override leo2 spawn as "x,y,z,R,P,Y" (empty = the '
                        'per-room pose authored for this world)'),
        DeclareLaunchArgument(
            'spawn_markers', default_value='auto',
            description='Spawn wall ArUco markers from the world\'s '
                        'mock_markers yaml. "auto" spawns only for worlds '
                        'that have no markers baked into the .sdf'),
        DeclareLaunchArgument(
            'gt_odom_tf', default_value='true',
            description='Publish Gazebo ground-truth odom->base_link on /tf. '
                        'Set false to run realistic wheel odometry instead '
                        '(scripts/sim_realism_odom.py)'),
        OpaqueFunction(function=launch_setup),
    ])
