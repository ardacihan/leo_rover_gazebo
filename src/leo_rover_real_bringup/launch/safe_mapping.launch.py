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

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    start_explorer = LaunchConfiguration("start_explorer")
    run_duration = LaunchConfiguration("run_duration")
    linear_speed = LaunchConfiguration("linear_speed")
    max_distance = LaunchConfiguration("max_distance")
    planned_turn_distance = LaunchConfiguration("planned_turn_distance")
    maximum_reverse_speed = LaunchConfiguration("maximum_reverse_speed")
    minimum_reverse_clearance = LaunchConfiguration("minimum_reverse_clearance")
    scan_topic = LaunchConfiguration("scan_topic")
    filtered_scan_topic = LaunchConfiguration("filtered_scan_topic")
    camera_scan_topic = LaunchConfiguration("camera_scan_topic")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    battery_topic = LaunchConfiguration("battery_topic")
    cmd_vel_request_topic = LaunchConfiguration("cmd_vel_request_topic")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("start_explorer", default_value="false"),
        DeclareLaunchArgument("run_duration", default_value="180.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.08"),
        DeclareLaunchArgument("max_distance", default_value="12.0"),
        DeclareLaunchArgument("planned_turn_distance", default_value="1.5"),
        DeclareLaunchArgument("maximum_reverse_speed", default_value="0.04"),
        DeclareLaunchArgument("minimum_reverse_clearance", default_value="0.75"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "filtered_scan_topic", default_value="/scan_self_filtered"
        ),
        DeclareLaunchArgument("camera_scan_topic", default_value="/camera/scan"),
        DeclareLaunchArgument("camera_x", default_value="0.065"),
        DeclareLaunchArgument("camera_y", default_value="-0.020"),
        DeclareLaunchArgument("camera_z", default_value="0.31"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "battery_topic", default_value="/rob_2/firmware/battery_averaged"
        ),
        DeclareLaunchArgument(
            "cmd_vel_request_topic", default_value="/cmd_vel_request"
        ),
        DeclareLaunchArgument("cmd_vel_in_topic", default_value="/cmd_vel_raw"),
        DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_static_transform",
            arguments=[
                "--x", lidar_x,
                "--y", lidar_y,
                "--z", lidar_z,
                "--yaw", lidar_yaw,
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "laser_frame",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_mount_static_transform",
            arguments=[
                "--x", camera_x, "--y", camera_y, "--z", camera_z,
                "--roll", camera_roll, "--pitch", camera_pitch,
                "--yaw", camera_yaw,
                "--frame-id", "base_footprint",
                "--child-frame-id", "camera_link",
            ],
            output="screen",
        ),
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            parameters=[{
                "output_frame": "camera_color_frame",
                "scan_height": 80,
                "scan_time": 0.0667,
                "range_min": 0.20,
                "range_max": 3.0,
            }],
            remappings=[
                ("depth", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("depth_camera_info", "/camera/camera/aligned_depth_to_color/camera_info"),
                ("scan", camera_scan_topic),
            ],
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="wheel_odom_tf.py",
            name="wheel_odom_tf",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_params],
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="safety_command_gate.py",
            name="safety_command_gate",
            parameters=[{
                "scan_topic": scan_topic,
                "battery_topic": battery_topic,
                "filtered_scan_topic": filtered_scan_topic,
                "camera_scan_topic": camera_scan_topic,
                "require_camera_scan": True,
                "cmd_vel_request_topic": cmd_vel_request_topic,
                "cmd_vel_raw_topic": cmd_vel_in_topic,
                "maximum_reverse_speed": maximum_reverse_speed,
                "minimum_reverse_clearance": minimum_reverse_clearance,
            }],
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
                    "observation_sources": ["scan", "camera_scan"],
                    "scan.topic": filtered_scan_topic,
                    "camera_scan.type": "scan",
                    "camera_scan.topic": camera_scan_topic,
                    "camera_scan.enabled": True,
                    "cmd_vel_in_topic": cmd_vel_in_topic,
                    "cmd_vel_out_topic": cmd_vel_out_topic,
                },
            ],
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
                "scan_topic": filtered_scan_topic,
                "camera_scan_topic": camera_scan_topic,
                "odom_topic": "/wheel_odom_integrated",
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
