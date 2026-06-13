from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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


def generate_launch_description():
    leo_gazebo_share = get_package_share_directory("leo_rover_gazebo")
    pkg_share = get_package_share_directory("multi_robot_shared_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot2_to_shared_x = LaunchConfiguration("robot2_to_shared_x")
    robot2_to_shared_y = LaunchConfiguration("robot2_to_shared_y")
    robot2_to_shared_yaw = LaunchConfiguration("robot2_to_shared_yaw")

    two_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(leo_gazebo_share, "launch", "two_robots.launch.py")
        )
    )

    # Each robot runs its own SLAM instance with namespaced map topics.
    # The output maps are merged by shared_map_merger.
    slam_leo1 = make_slam_node("leo1", use_sim_time)
    slam_leo2 = make_slam_node("leo2", use_sim_time)

    lidar_tf_leo1 = make_lidar_tf_node("leo1", use_sim_time)
    lidar_tf_leo2 = make_lidar_tf_node("leo2", use_sim_time)

    map_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_frame_tf_leo1_leo2",
        arguments=[
            "2.36", "-11.27", "0.0",
            "0", "0", "0",
            "leo1/map",
            "leo2/map",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Robot2 map alignment into leo1/shared frame (matches leo2 spawn in Gazebo).
        DeclareLaunchArgument("robot2_to_shared_x", default_value="2.36"),
        DeclareLaunchArgument("robot2_to_shared_y", default_value="-11.27"),
        DeclareLaunchArgument("robot2_to_shared_yaw", default_value="0.0"),

        two_robots_launch,
        odom_tf_broadcaster,
        lidar_tf_leo1,
        lidar_tf_leo2,
        slam_leo1,
        slam_leo2,
        map_frame_tf,
        shared_map_merger,
        robot_state_registry,
    ])
