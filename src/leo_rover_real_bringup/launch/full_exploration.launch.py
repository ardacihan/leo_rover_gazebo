"""Frontier-driven exploration with debug capture.

Layers on top of a running `safe_mapping.launch.py` and `imu_odometry.launch.py`
rather than replacing them, so SLAM, the fused scans and the safety chain keep
running untouched across exploration attempts.

    ros2 launch leo_rover_real_bringup full_exploration.launch.py \
        run_duration:=600.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_duration = LaunchConfiguration("run_duration")
    linear_speed = LaunchConfiguration("linear_speed")
    start_explorer = LaunchConfiguration("start_explorer")
    start_recorder = LaunchConfiguration("start_recorder")
    record_video = LaunchConfiguration("record_video")
    incident_directory = LaunchConfiguration("incident_directory")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    return LaunchDescription([
        DeclareLaunchArgument("run_duration", default_value="600.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.08"),
        DeclareLaunchArgument("start_explorer", default_value="true"),
        DeclareLaunchArgument("start_recorder", default_value="true"),
        DeclareLaunchArgument("record_video", default_value="true"),
        DeclareLaunchArgument(
            "incident_directory", default_value="/home/jetson-04/leo_incidents"
        ),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan_collision_fused"
        ),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        # Started first so the opening moments of a run are already covered.
        Node(
            package="leo_rover_real_bringup",
            executable="incident_recorder.py",
            name="incident_recorder",
            parameters=[{
                "output_directory": incident_directory,
                "fused_scan_topic": scan_topic,
                "odom_topic": odom_topic,
                "record_video": record_video,
            }],
            condition=IfCondition(start_recorder),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            parameters=[{
                "run_duration": run_duration,
                "linear_speed": linear_speed,
                "scan_topic": scan_topic,
                "odom_topic": odom_topic,
            }],
            condition=IfCondition(start_explorer),
            output="screen",
            emulate_tty=True,
        ),
    ])
