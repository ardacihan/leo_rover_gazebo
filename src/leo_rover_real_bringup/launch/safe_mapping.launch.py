import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("leo_rover_real_bringup")
    slam_params = os.path.join(package_dir, "config", "slam_params.yaml")
    lidar_only_params = os.path.join(
        package_dir, "config", "collision_monitor_params.yaml"
    )
    lidar_and_camera_params = os.path.join(
        package_dir, "config", "collision_monitor_camera_params.yaml"
    )

    use_camera = LaunchConfiguration("use_camera_collision")

    # Collision Monitor rejects a configured source that never arrives, so the
    # source list and the depth pipeline have to be switched together.
    collision_params = PythonExpression([
        "'", lidar_and_camera_params, "'",
        " if '", use_camera, "'.lower() in ('true', '1') else ",
        "'", lidar_only_params, "'",
    ])

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    start_explorer = LaunchConfiguration("start_explorer")
    start_lidar_tf = LaunchConfiguration("start_lidar_tf")
    start_wheel_odom_tf = LaunchConfiguration("start_wheel_odom_tf")
    start_slam = LaunchConfiguration("start_slam")
    odom_topic = LaunchConfiguration("odom_topic")
    run_duration = LaunchConfiguration("run_duration")
    max_distance = LaunchConfiguration("max_distance")
    planned_turn_distance = LaunchConfiguration("planned_turn_distance")
    maximum_reverse_speed = LaunchConfiguration("maximum_reverse_speed")
    minimum_reverse_clearance = LaunchConfiguration("minimum_reverse_clearance")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_yaw_offset = LaunchConfiguration("scan_yaw_offset")
    start_scan_filter = LaunchConfiguration("start_scan_filter")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_filter_sectors = LaunchConfiguration("scan_filter_sectors")
    scan_filter_max_range = LaunchConfiguration("scan_filter_max_range")
    battery_topic = LaunchConfiguration("battery_topic")
    minimum_battery_voltage = LaunchConfiguration("minimum_battery_voltage")
    cmd_vel_request_topic = LaunchConfiguration("cmd_vel_request_topic")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")

    return LaunchDescription([
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("start_explorer", default_value="false"),
        # Rovers whose installed stack already owns these must set them false.
        # Rover 4 publishes base_link -> laser_frame from /etc/ros and
        # odom -> base_footprint from leo_nav_bridge; duplicating either
        # corrupts the scan poses.
        DeclareLaunchArgument("start_lidar_tf", default_value="true"),
        DeclareLaunchArgument("start_wheel_odom_tf", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument(
            "odom_topic", default_value="/wheel_odom_integrated"
        ),
        DeclareLaunchArgument("run_duration", default_value="180.0"),
        DeclareLaunchArgument("max_distance", default_value="12.0"),
        DeclareLaunchArgument("planned_turn_distance", default_value="1.5"),
        DeclareLaunchArgument("maximum_reverse_speed", default_value="0.04"),
        DeclareLaunchArgument("minimum_reverse_clearance", default_value="0.75"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        # Self-occlusion masking.  raw_scan_topic is the driver's scan;
        # scan_topic is what every consumer reads.  To enable, set
        # start_scan_filter:=true scan_topic:=/scan_filtered and list the
        # blocked sectors in base-frame degrees as "low:high,low:high".
        # --- Depth-camera collision source -------------------------------
        # The RealSense publishes its own frames rooted at camera_link, but
        # nothing connects them to the robot, so Collision Monitor cannot
        # transform depth data until this static transform exists.  MEASURE
        # the mount; the defaults below are placeholders, not a calibration.
        DeclareLaunchArgument("use_camera_collision", default_value="false"),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "depth_image_topic",
            default_value="/camera/camera/depth/image_rect_raw",
        ),
        DeclareLaunchArgument(
            "depth_info_topic",
            default_value="/camera/camera/depth/camera_info",
        ),
        DeclareLaunchArgument("depth_scan_topic", default_value="/camera_scan"),
        # Height band kept as obstacles, metres above the ground.  The lower
        # bound is what rejects the floor: measured on Rover 4, a row-based
        # conversion reported the floor as a wall at 1.44 m.  The upper bound
        # ignores door frames and ceilings the rover passes under.
        DeclareLaunchArgument("min_obstacle_height", default_value="0.06"),
        DeclareLaunchArgument("max_obstacle_height", default_value="0.60"),
        # Pixel decimation.  2 keeps a quarter of the pixels, which leaves
        # Collision Monitor the CPU it needs to not drop its sources.
        DeclareLaunchArgument("depth_pixel_step", default_value="2"),
        # The D456 cannot measure closer than roughly 0.45 m.
        DeclareLaunchArgument("depth_range_min", default_value="0.45"),
        DeclareLaunchArgument("depth_range_max", default_value="6.0"),
        DeclareLaunchArgument("start_scan_filter", default_value="false"),
        DeclareLaunchArgument("raw_scan_topic", default_value="/scan"),
        DeclareLaunchArgument("scan_filter_sectors", default_value=""),
        DeclareLaunchArgument("scan_filter_max_range", default_value="0.45"),
        # Yaw from the base frame to the LIDAR frame, in radians.  The gate and
        # explorer read raw scan angles, so a LIDAR that does not face forward
        # mirrors their front/rear sectors.  Rover 4 needs 3.14159.
        DeclareLaunchArgument("scan_yaw_offset", default_value="0.0"),
        DeclareLaunchArgument(
            "battery_topic", default_value="/firmware/battery_averaged"
        ),
        DeclareLaunchArgument("minimum_battery_voltage", default_value="5.0"),
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
            condition=IfCondition(start_lidar_tf),
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="wheel_odom_tf.py",
            name="wheel_odom_tf",
            condition=IfCondition(start_wheel_odom_tf),
            output="screen",
            emulate_tty=True,
        ),
        # Connects the RealSense subtree (rooted at camera_link) to the robot.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_static_transform",
            arguments=[
                "--x", LaunchConfiguration("camera_x"),
                "--y", LaunchConfiguration("camera_y"),
                "--z", LaunchConfiguration("camera_z"),
                "--roll", LaunchConfiguration("camera_roll"),
                "--pitch", LaunchConfiguration("camera_pitch"),
                "--yaw", LaunchConfiguration("camera_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", "camera_link",
            ],
            condition=IfCondition(use_camera),
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="depth_obstacle_scan.py",
            name="depth_obstacle_scan",
            parameters=[{
                "depth_topic": LaunchConfiguration("depth_image_topic"),
                "info_topic": LaunchConfiguration("depth_info_topic"),
                "output_topic": LaunchConfiguration("depth_scan_topic"),
                "target_frame": "base_footprint",
                "min_obstacle_height": LaunchConfiguration("min_obstacle_height"),
                "max_obstacle_height": LaunchConfiguration("max_obstacle_height"),
                "range_min": LaunchConfiguration("depth_range_min"),
                "range_max": LaunchConfiguration("depth_range_max"),
                "pixel_step": LaunchConfiguration("depth_pixel_step"),
            }],
            condition=IfCondition(use_camera),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="scan_self_filter.py",
            name="scan_self_filter",
            parameters=[{
                "input_topic": raw_scan_topic,
                "output_topic": scan_topic,
                "scan_yaw_offset": scan_yaw_offset,
                "exclusion_sectors": scan_filter_sectors,
                "exclusion_max_range": scan_filter_max_range,
            }],
            condition=IfCondition(start_scan_filter),
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            # The override must follow the file so the filtered scan wins;
            # otherwise SLAM maps the rover's own body.
            parameters=[slam_params, {"scan_topic": scan_topic}],
            condition=IfCondition(start_slam),
            output="screen",
        ),
        Node(
            package="leo_rover_real_bringup",
            executable="safety_command_gate.py",
            name="safety_command_gate",
            parameters=[{
                "scan_topic": scan_topic,
                "scan_yaw_offset": scan_yaw_offset,
                "battery_topic": battery_topic,
                "minimum_battery_voltage": minimum_battery_voltage,
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
                    "depth_scan.topic": LaunchConfiguration("depth_scan_topic"),
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
                "scan_yaw_offset": scan_yaw_offset,
                "battery_topic": battery_topic,
                "minimum_battery_voltage": minimum_battery_voltage,
                "odom_topic": odom_topic,
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
