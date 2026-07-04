from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

from multi_robot_shared_mapping.apriltag_inline_model import (
    load_tag_spawn_entries,
    make_inline_tag_sdf,
)


def make_gazebo_and_robots():
    """Gazebo + leo1/leo2 without semantic vision or ArUco markers."""
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("leo_rover_description")

    xacro_file = os.path.join(
        pkg_description, "urdf", "leo_rover_with_sensors.urdf.xacro"
    )
    world_path = "/ros2_ws/src/husarion_gz_worlds/worlds/husarion_office.sdf"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    entities = [gz_sim, clock_bridge]

    spawn_poses = {
        "leo1": ("0.0", "0.0", "0.2", "0.0", "0.0", "0.0"),
        "leo2": ("2.36", "-11.27", "0.05", "0.0", "0.0", "0.0"),
    }

    for robot_ns in ("leo1", "leo2"):
        spawn_x, spawn_y, spawn_z, spawn_R, spawn_P, spawn_Y = spawn_poses[robot_ns]

        robot_desc = Command([
            "xacro", " ", xacro_file, " ", "robot_ns:=", robot_ns
        ])

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_ns,
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_desc, value_type=str),
                "use_sim_time": True,
                "publish_frequency": 50.0,
            }],
            remappings=[("/tf", "/tf"), ("tf_static", "/tf_static")],
            output="screen",
        )

        gpu_lidar_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"gpu_lidar_tf_{robot_ns}",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                f"{robot_ns}/sensor_lidar_link",
                f"{robot_ns}/base_footprint/gpu_lidar",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )

        spawn = TimerAction(
            period=5.0,
            actions=[Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{robot_ns}",
                arguments=[
                    "-name", robot_ns,
                    "-topic", f"/{robot_ns}/robot_description",
                    "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
                    "-R", spawn_R, "-P", spawn_P, "-Y", spawn_Y,
                ],
                output="screen",
            )],
        )

        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot_ns,
            name=f"bridge_{robot_ns}",
            arguments=[
                f"/{robot_ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/{robot_ns}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                f"/{robot_ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/{robot_ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/{robot_ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/{robot_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"/{robot_ns}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                f"/model/{robot_ns}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
                f"/model/{robot_ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                f"/{robot_ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            remappings=[(f"/model/{robot_ns}/tf", "/tf")],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )

        entities += [rsp, spawn, bridge, gpu_lidar_tf]

    return entities


def make_lidar_tf_node(robot_name, use_sim_time):
    # Combined URDF offset: base_footprint -> base_link (z=0.19783)
    # plus base_link -> sensor_lidar_link (x=0.2, z=0.4).
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"lidar_tf_{robot_name}",
        arguments=[
            "0.2", "0", "0.59783",
            "0", "0", "0",
            f"{robot_name}/base_footprint",
            f"{robot_name}/sensor_lidar_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )


def make_slam_params(robot_name):
    return {
        "odom_frame": f"{robot_name}/odom",
        "map_frame": f"{robot_name}/map",
        "base_frame": f"{robot_name}/base_footprint",
        "scan_topic": f"/{robot_name}/scan",
        "mode": "mapping",
        "debug_logging": False,
        "throttle_scans": 1,
        "transform_publish_period": 0.02,
        "map_update_interval": 1.0,
        "resolution": 0.05,
        "max_laser_range": 12.0,
        "minimum_time_interval": 0.5,
        "transform_timeout": 0.5,
        "tf_buffer_duration": 30.0,
        "stack_size_to_use": 40000000,
        "enable_interactive_mode": True,
    }


def make_slam_node(robot_name, use_sim_time):
    return Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name=f"slam_toolbox_{robot_name}",
        output="screen",
        parameters=[
            make_slam_params(robot_name),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/map", f"/{robot_name}/map"),
            ("/map_metadata", f"/{robot_name}/map_metadata"),
            ("/scan", f"/{robot_name}/scan"),
        ],
    )


def make_apriltag_spawn_actions(assets_dir: str, landmarks_path: str):
    """Spawn inline SDF tag planes (no model:// URIs)."""
    actions = []
    for index, (tag_name, x, y, z, roll, pitch, yaw, texture_path, tag_size) in enumerate(
        load_tag_spawn_entries(landmarks_path, assets_dir)
    ):
        tag_id = int(tag_name.split("_")[1])
        inline_sdf = make_inline_tag_sdf(tag_name, tag_id, texture_path, tag_size)
        actions.append(
            TimerAction(
                period=6.0 + index * 0.5,
                actions=[Node(
                    package="ros_gz_sim",
                    executable="create",
                    name=f"spawn_{tag_name}",
                    arguments=[
                        "-name", tag_name,
                        "-x", x, "-y", y, "-z", z,
                        "-R", roll, "-P", pitch, "-Y", yaw,
                        "-string", inline_sdf,
                    ],
                    output="screen",
                )],
            )
        )
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("multi_robot_shared_mapping")
    assets_dir = os.path.join(pkg_share, "assets", "apriltags")
    landmarks_path = os.path.join(pkg_share, "config", "apriltag_landmarks.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot2_to_shared_x = LaunchConfiguration("robot2_to_shared_x")
    robot2_to_shared_y = LaunchConfiguration("robot2_to_shared_y")
    robot2_to_shared_yaw = LaunchConfiguration("robot2_to_shared_yaw")
    enable_apriltag_detection = LaunchConfiguration("enable_apriltag_detection")
    enable_tag_alignment = LaunchConfiguration("enable_tag_alignment")
    enable_map_alignment = LaunchConfiguration("enable_map_alignment")
    alignment_mode = LaunchConfiguration("alignment_mode")
    enable_alignment_evaluation = LaunchConfiguration("enable_alignment_evaluation")
    tag_cache_timeout_sec = LaunchConfiguration("tag_cache_timeout_sec")
    min_alignment_confidence = LaunchConfiguration("min_alignment_confidence")
    landmark_persistence = LaunchConfiguration("landmark_persistence")

    gazebo_and_robots = make_gazebo_and_robots()

    slam_leo1 = make_slam_node("leo1", use_sim_time)
    slam_leo2 = make_slam_node("leo2", use_sim_time)

    lidar_tf_leo1 = make_lidar_tf_node("leo1", use_sim_time)
    lidar_tf_leo2 = make_lidar_tf_node("leo2", use_sim_time)

    map_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_frame_tf_leo1_leo2",
        arguments=[
            robot2_to_shared_x,
            robot2_to_shared_y,
            "0.0",
            "0", "0", robot2_to_shared_yaw,
            "leo1/map",
            "leo2/map",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=LaunchConfigurationEquals("alignment_mode", "fixed"),
    )

    shared_map_merger = Node(
        package="multi_robot_shared_mapping",
        executable="shared_map_merger",
        name="shared_map_merger",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map1_topic": "/leo1/map",
            "map2_topic": "/leo2/map",
            "shared_map_topic": "/shared_map",
            "shared_frame_id": "leo1/map",
            "alignment_mode": alignment_mode,
            "estimated_transform_topic": "/estimated_transform/leo2_to_leo1",
            "map_transform_topic": "/map_based_transform/leo2_to_leo1",
            "min_alignment_confidence": min_alignment_confidence,
            "robot2_to_shared_x": robot2_to_shared_x,
            "robot2_to_shared_y": robot2_to_shared_y,
            "robot2_to_shared_yaw": robot2_to_shared_yaw,
        }],
    )

    robot_state_registry = Node(
        package="multi_robot_shared_mapping",
        executable="robot_state_registry",
        name="robot_state_registry",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    odom_tf_broadcaster = Node(
        package="multi_robot_shared_mapping",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    apriltag_detection_node = Node(
        package="multi_robot_shared_mapping",
        executable="apriltag_detection_node",
        name="apriltag_detection_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "tag_size_m": 0.35,
        }],
        condition=IfCondition(enable_apriltag_detection),
    )

    tag_based_map_aligner = Node(
        package="multi_robot_shared_mapping",
        executable="tag_based_map_aligner",
        name="tag_based_map_aligner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "landmark_persistence": landmark_persistence,
            "tag_cache_timeout_sec": tag_cache_timeout_sec,
            "ground_truth_x": robot2_to_shared_x,
            "ground_truth_y": robot2_to_shared_y,
            "ground_truth_yaw": robot2_to_shared_yaw,
            "compare_to_ground_truth": enable_alignment_evaluation,
        }],
        condition=IfCondition(enable_tag_alignment),
    )

    map_based_aligner = Node(
        package="multi_robot_shared_mapping",
        executable="map_based_aligner",
        name="map_based_aligner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "alignment_mode": alignment_mode,
            "min_alignment_confidence": min_alignment_confidence,
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_tag_alignment, "' == 'true' or '",
            enable_map_alignment, "' == 'true'",
        ])),
    )

    apriltag_spawn_group = GroupAction(
        condition=IfCondition(enable_apriltag_detection),
        actions=make_apriltag_spawn_actions(assets_dir, landmarks_path),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot2_to_shared_x", default_value="2.36"),
        DeclareLaunchArgument("robot2_to_shared_y", default_value="-11.27"),
        DeclareLaunchArgument("robot2_to_shared_yaw", default_value="0.0"),
        DeclareLaunchArgument("enable_apriltag_detection", default_value="false"),
        DeclareLaunchArgument("enable_tag_alignment", default_value="false"),
        DeclareLaunchArgument("enable_map_alignment", default_value="false"),
        # fixed | tag | map | hybrid ("estimated" kept as alias for tag)
        DeclareLaunchArgument("alignment_mode", default_value="fixed"),
        DeclareLaunchArgument("enable_alignment_evaluation", default_value="false"),
        DeclareLaunchArgument("tag_cache_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("min_alignment_confidence", default_value="0.5"),
        # Persistent landmark map: tags are never forgotten once seen.
        DeclareLaunchArgument("landmark_persistence", default_value="true"),

        *gazebo_and_robots,
        odom_tf_broadcaster,
        lidar_tf_leo1,
        lidar_tf_leo2,
        slam_leo1,
        slam_leo2,
        map_frame_tf,
        shared_map_merger,
        robot_state_registry,
        apriltag_spawn_group,
        apriltag_detection_node,
        tag_based_map_aligner,
        map_based_aligner,
    ])
