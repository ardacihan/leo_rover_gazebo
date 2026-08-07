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
    max_distance = LaunchConfiguration("max_distance")
    planned_turn_distance = LaunchConfiguration("planned_turn_distance")
    maximum_reverse_speed = LaunchConfiguration("maximum_reverse_speed")
    minimum_reverse_clearance = LaunchConfiguration("minimum_reverse_clearance")
    scan_topic = LaunchConfiguration("scan_topic")
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
        DeclareLaunchArgument("max_distance", default_value="12.0"),
        DeclareLaunchArgument("planned_turn_distance", default_value="1.5"),
        DeclareLaunchArgument("maximum_reverse_speed", default_value="0.04"),
        DeclareLaunchArgument("minimum_reverse_clearance", default_value="0.75"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
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
                "--child-frame-id", "laser",
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
                    "scan.topic": scan_topic,
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
                "max_distance": max_distance,
                "planned_turn_distance": planned_turn_distance,
                "scan_topic": scan_topic,
                "odom_topic": "/wheel_odom_integrated",
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
