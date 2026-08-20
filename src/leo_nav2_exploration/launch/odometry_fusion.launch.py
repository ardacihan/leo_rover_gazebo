"""Wheel + gyro odometry fusion for the physical rover.

    ros2 launch leo_nav2_exploration odometry_fusion.launch.py

Brings up the firmware-IMU bridge and a `robot_localization` EKF that owns
`odom -> base_footprint`.

**Stop the rover's existing odometry TF publisher first.** `wheel_odom_tf.py`
in `leo_rover_real_bringup` publishes the same transform. Two publishers of one
transform is not an error in tf2 -- consumers just see the pose flip between
two estimates, which looks exactly like a SLAM failure. `config/real/ekf.yaml`
documents the check.

Launch this *before* the navigation overlay, so SLAM starts against a stable
`odom` frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('leo_nav2_exploration')
    ekf_params = os.path.join(share, 'config', 'real', 'ekf.yaml')

    imu_bridge = Node(
        package='leo_nav2_exploration',
        executable='leo_imu_bridge',
        name='leo_imu_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_imu_bridge')),
        parameters=[{
            'input_topic': LaunchConfiguration('firmware_imu_topic'),
            'output_topic': LaunchConfiguration('imu_topic'),
            'frame_id': LaunchConfiguration('imu_frame'),
            'calibration_samples': LaunchConfiguration('calibration_samples'),
        }],
    )

    # The firmware IMU sits on the mainboard, not at base_footprint. Only the
    # z gyro is fused and a pure translation does not change a rate, so the
    # offset does not affect the filter -- but tf2 still needs the frame to
    # exist or every IMU message is dropped as untransformable.
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        condition=IfCondition(LaunchConfiguration('publish_imu_tf')),
        arguments=[
            '--x', LaunchConfiguration('imu_x'),
            '--y', LaunchConfiguration('imu_y'),
            '--z', LaunchConfiguration('imu_z'),
            '--frame-id', 'base_footprint',
            '--child-frame-id', LaunchConfiguration('imu_frame'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_imu_bridge', default_value='true'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('firmware_imu_topic', default_value='/firmware/imu'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('calibration_samples', default_value='200'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.09'),
        imu_bridge,
        imu_tf,
        ekf,
    ])
