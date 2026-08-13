import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("leo_rover_real_bringup")
    slam_params = os.path.join(package_dir, "config", "slam_params.yaml")
    collision_params = os.path.join(
        package_dir, "config", "collision_monitor_params.yaml"
    )

    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_parent_frame = LaunchConfiguration("lidar_parent_frame")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    publish_camera_tf = LaunchConfiguration("publish_camera_tf")
    start_sensor_fusion = LaunchConfiguration("start_sensor_fusion")
    start_slam = LaunchConfiguration("start_slam")
    start_safety = LaunchConfiguration("start_safety")
    record_mapping_artifacts = LaunchConfiguration("record_mapping_artifacts")
    start_coverage_reporter = LaunchConfiguration("start_coverage_reporter")
    start_explorer = LaunchConfiguration("start_explorer")

    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    lidar_base_scan_topic = LaunchConfiguration("lidar_base_scan_topic")
    camera_collision_scan_topic = LaunchConfiguration(
        "camera_collision_scan_topic"
    )
    camera_slam_scan_topic = LaunchConfiguration("camera_slam_scan_topic")
    collision_fused_scan_topic = LaunchConfiguration(
        "collision_fused_scan_topic"
    )
    slam_fused_scan_topic = LaunchConfiguration("slam_fused_scan_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    run_duration = LaunchConfiguration("run_duration")
    linear_speed = LaunchConfiguration("linear_speed")
    max_distance = LaunchConfiguration("max_distance")
    planned_turn_distance = LaunchConfiguration("planned_turn_distance")
    maximum_reverse_speed = LaunchConfiguration("maximum_reverse_speed")
    minimum_reverse_clearance = LaunchConfiguration("minimum_reverse_clearance")
    battery_topic = LaunchConfiguration("battery_topic")
    cmd_vel_request_topic = LaunchConfiguration("cmd_vel_request_topic")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    artifact_output_directory = LaunchConfiguration("artifact_output_directory")
    artifact_prefix = LaunchConfiguration("artifact_prefix")

    return LaunchDescription([
        # Rover 4's boot services normally already own LIDAR and odometry TF.
        # Publishing duplicates is therefore opt-in.
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        DeclareLaunchArgument("publish_odom_tf", default_value="false"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
        DeclareLaunchArgument("start_sensor_fusion", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_safety", default_value="true"),
        DeclareLaunchArgument("record_mapping_artifacts", default_value="true"),
        DeclareLaunchArgument("start_coverage_reporter", default_value="true"),
        DeclareLaunchArgument("start_explorer", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("odom_topic", default_value="/wheel_odom"),
        DeclareLaunchArgument("lidar_parent_frame", default_value="base_footprint"),
        # Measured on Rover 4, 2026-08-13: base_footprint <- laser_frame.
        DeclareLaunchArgument("lidar_x", default_value="0.0775"),
        # Operator-measured 2026-08-13: the lidar sits 0.04 m to the rover's
        # left. lidar-tf.service publishes y=0, so the boot transform carries a
        # 4 cm lateral error. Confirmed independently: the mast appears at
        # y=-0.074 under the y=0 assumption but is physically at about -0.04.
        DeclareLaunchArgument("lidar_y", default_value="0.04"),
        DeclareLaunchArgument("lidar_z", default_value="0.2458"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.14159"),
        # Approximate Rover 4 camera mount. Validate these measurements before
        # treating camera-derived geometry as navigation-grade.
        # z/roll/pitch measured 2026-08-13 by fitting the floor plane in eight
        # aligned depth frames (89.5% inliers): the mast camera sits 0.393 m above
        # the floor and is pitched 12 deg down, which the previous 0.31 m /
        # level defaults missed entirely.  x/y still need a tape measure.
        # camera_link is the depth imager; the colour lens it is aligned to sits
        # 0.059 m to its right. Operator measured the camera 0.04 m right of
        # centre and 0.0175 m behind the lidar axis, so camera_link goes
        # slightly left of centre to place the colour lens at y=-0.04.
        DeclareLaunchArgument("camera_x", default_value="0.060"),
        DeclareLaunchArgument("camera_y", default_value="0.019"),
        DeclareLaunchArgument("camera_z", default_value="0.393"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.209"),
        # -2 deg, recovered by aligning height-filtered depth against the lidar
        # (residual 0.083 m, against 0.230 m at 0 deg and 0.652 m at +2 deg).
        DeclareLaunchArgument("camera_yaw", default_value="-0.035"),
        DeclareLaunchArgument("raw_scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "lidar_base_scan_topic", default_value="/scan_lidar_base"
        ),
        DeclareLaunchArgument(
            "camera_collision_scan_topic",
            default_value="/camera/scan_collision",
        ),
        DeclareLaunchArgument(
            "camera_slam_scan_topic", default_value="/camera/scan_slam"
        ),
        DeclareLaunchArgument(
            "collision_fused_scan_topic",
            default_value="/scan_collision_fused",
        ),
        DeclareLaunchArgument(
            "slam_fused_scan_topic", default_value="/scan_slam_fused"
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/aligned_depth_to_color/camera_info",
        ),
        DeclareLaunchArgument("run_duration", default_value="180.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.08"),
        DeclareLaunchArgument("max_distance", default_value="12.0"),
        DeclareLaunchArgument("planned_turn_distance", default_value="1.5"),
        DeclareLaunchArgument("maximum_reverse_speed", default_value="0.0"),
        DeclareLaunchArgument("minimum_reverse_clearance", default_value="0.75"),
        DeclareLaunchArgument(
            "battery_topic", default_value="/rob_2/firmware/battery_averaged"
        ),
        DeclareLaunchArgument(
            "cmd_vel_request_topic", default_value="/cmd_vel_request"
        ),
        DeclareLaunchArgument("cmd_vel_in_topic", default_value="/cmd_vel_raw"),
        DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("artifact_output_directory", default_value=""),
        DeclareLaunchArgument("artifact_prefix", default_value="leo_room"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_static_transform",
            arguments=[
                "--x", lidar_x,
                "--y", lidar_y,
                "--z", lidar_z,
                "--roll", lidar_roll,
                "--pitch", lidar_pitch,
                "--yaw", lidar_yaw,
                "--frame-id", lidar_parent_frame,
                "--child-frame-id", "laser_frame",
            ],
            condition=IfCondition(publish_lidar_tf),
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_mount_static_transform",
            arguments=[
                "--x", camera_x,
                "--y", camera_y,
                "--z", camera_z,
                "--roll", camera_roll,
                "--pitch", camera_pitch,
                "--yaw", camera_yaw,
                "--frame-id", base_frame,
                "--child-frame-id", "camera_link",
            ],
            condition=IfCondition(publish_camera_tf),
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="depth_height_filter.py",
            name="depth_height_filter",
            parameters=[{
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "collision_scan_topic": camera_collision_scan_topic,
                "slam_scan_topic": camera_slam_scan_topic,
                "base_frame": base_frame,
            }],
            condition=IfCondition(start_sensor_fusion),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="scan_fusion.py",
            name="scan_fusion",
            parameters=[{
                "lidar_scan_topic": raw_scan_topic,
                "camera_collision_scan_topic": camera_collision_scan_topic,
                "camera_slam_scan_topic": camera_slam_scan_topic,
                "lidar_base_scan_topic": lidar_base_scan_topic,
                "collision_fused_scan_topic": collision_fused_scan_topic,
                "slam_fused_scan_topic": slam_fused_scan_topic,
                "base_frame": base_frame,
            }],
            condition=IfCondition(start_sensor_fusion),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="wheel_odom_tf.py",
            name="wheel_odom_tf",
            parameters=[{
                "input_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
            }],
            condition=IfCondition(publish_odom_tf),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[
                slam_params,
                {
                    "scan_topic": slam_fused_scan_topic,
                    "base_frame": base_frame,
                    "odom_frame": odom_frame,
                },
            ],
            condition=IfCondition(start_slam),
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="mapping_artifact_recorder.py",
            name="mapping_artifact_recorder",
            parameters=[{
                "map_frame": "map",
                "base_frame": base_frame,
                "output_directory": artifact_output_directory,
                "artifact_prefix": artifact_prefix,
            }],
            condition=IfCondition(record_mapping_artifacts),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="map_coverage_reporter.py",
            name="map_coverage_reporter",
            parameters=[{"map_topic": "/map"}],
            condition=IfCondition(start_coverage_reporter),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="safety_command_gate.py",
            name="safety_command_gate",
            parameters=[{
                "scan_topic": collision_fused_scan_topic,
                "camera_scan_topic": camera_collision_scan_topic,
                "require_camera_scan": True,
                "publish_filtered_scan": False,
                "self_mask_radius": 0.0,
                "base_frame": base_frame,
                "scan_yaw_offset": 0.0,
                "odom_topic": odom_topic,
                "battery_topic": battery_topic,
                "cmd_vel_request_topic": cmd_vel_request_topic,
                "cmd_vel_raw_topic": cmd_vel_in_topic,
                "cmd_vel_output_topic": cmd_vel_out_topic,
                # leo_real keeps its disabled supervisor node alive on Rover 4;
                # its endpoint is allowed, while controller_server/teleop are not.
                "allowed_cmd_vel_output_publishers": [
                    "collision_monitor", "robot_supervisor_rgb"
                ],
                "maximum_reverse_speed": maximum_reverse_speed,
                "minimum_reverse_clearance": minimum_reverse_clearance,
            }],
            condition=IfCondition(start_safety),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            parameters=[
                collision_params,
                {
                    "observation_sources": ["fused_scan"],
                    "fused_scan.type": "scan",
                    "fused_scan.topic": collision_fused_scan_topic,
                    "fused_scan.enabled": True,
                    "cmd_vel_in_topic": cmd_vel_in_topic,
                    "cmd_vel_out_topic": cmd_vel_out_topic,
                },
            ],
            condition=IfCondition(start_safety),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["collision_monitor"],
            }],
            condition=IfCondition(start_safety),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="safe_room_explorer.py",
            name="safe_room_explorer",
            parameters=[{
                "run_duration": run_duration,
                "linear_speed": linear_speed,
                "max_distance": max_distance,
                "planned_turn_distance": planned_turn_distance,
                "scan_topic": collision_fused_scan_topic,
                "scan_yaw_offset": 0.0,
                "camera_scan_topic": camera_collision_scan_topic,
                "camera_scan_yaw_offset": 0.0,
                "base_frame": base_frame,
                "odom_topic": odom_topic,
                "battery_topic": battery_topic,
                "cmd_vel_request_topic": cmd_vel_request_topic,
                "cmd_vel_output_topic": cmd_vel_out_topic,
                "reverse_speed": maximum_reverse_speed,
                "minimum_reverse_clearance": minimum_reverse_clearance,
            }],
            condition=IfCondition(start_explorer),
            output="screen",
            emulate_tty=True,
        ),
    ])
