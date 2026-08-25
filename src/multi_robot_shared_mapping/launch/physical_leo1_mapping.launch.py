from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")

    # Physical SLAMTEC RPLIDAR C1.
    lidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node_leo1",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 460800,
                "frame_id": "leo1/sensor_lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
        remappings=[
            ("scan", "/leo1/scan"),
        ],
    )

    # Approximate transform based on your mounting position.
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_leo1",
        output="screen",
        arguments=[
            "--x", "0.075",
            "--y", "0.035",
            "--z", "0.22763",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "leo1/base_footprint",
            "--child-frame-id", "leo1/sensor_lidar_link",
        ],
    )

    # Convert Leo's physical odometry into your project's namespaced TF.
    odom_tf = Node(
        package="multi_robot_shared_mapping",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster_physical",
        output="screen",
        parameters=[
            {"use_sim_time": False},
        ],
        remappings=[
            ("/leo1/odom", odom_topic),
        ],
    )

    # SLAM Toolbox for the physical rover.
    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_leo1",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "odom_frame": "leo1/odom",
                "map_frame": "leo1/map",
                "base_frame": "leo1/base_footprint",
                "scan_topic": "/leo1/scan",
                "mode": "mapping",
                "debug_logging": False,
                "throttle_scans": 1,
                "transform_publish_period": 0.02,
                "map_update_interval": 1.0,
                "resolution": 0.05,
                "max_laser_range": 12.0,
                "minimum_time_interval": 0.1,
                "transform_timeout": 0.5,
                "tf_buffer_duration": 30.0,
                "stack_size_to_use": 40000000,
                "enable_interactive_mode": True,
            }
        ],
        remappings=[
            ("/scan", "/leo1/scan"),
            ("/map", "/leo1/map"),
            ("/map_metadata", "/leo1/map_metadata"),
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating physical leo1 SLAM"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Physical Leo odometry topic",
            ),
            lidar,
            lidar_tf,
            odom_tf,
            slam,
            configure_slam,
            activate_slam,
        ]
    )