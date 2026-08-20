"""Static sensor-mount transforms for the physical rover.

The rover's RealSense driver owns camera_link -> camera_*_optical_frame but
nothing connects camera_link to the base tree, so the depth cloud is
untransformable without this. Kept out of navigation_overlay.launch.py on
purpose: that file must not own TF (see test_launch_source_contracts), and TF
must have exactly one owner. Do not start this if another node (e.g.
safe_mapping.launch.py with publish_camera_tf:=true) already publishes
base_link -> camera_link.

Camera values calibrated on rover 4, 2026-08-20: floor-plane fit over six
depth frames (84% inliers, 4 mm rms) gave optical height 0.388 m above the
floor (tape measure: 0.39) and 10.97 deg pitch down; base_footprint ->
base_link is 0.19783 m, hence z = 0.1905 from base_link. x = 0.060 was tape
measured. y: the operator measured the camera 0.035 m right of centre;
camera_link is the depth imager and the colour lens sits 0.059 m to its
right, so camera_link lands at +0.024 (left of centre). Yaw -0.035 is the
2026-08-13 empirical depth-to-lidar alignment; a floor fit cannot observe
yaw.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'publish_camera_tf',
                default_value='true',
                description='Publish the calibrated base_link -> camera_link mount transform.',
            ),
            DeclareLaunchArgument('camera_x', default_value='0.060'),
            DeclareLaunchArgument('camera_y', default_value='0.024'),
            DeclareLaunchArgument('camera_z', default_value='0.1905'),
            DeclareLaunchArgument('camera_roll', default_value='-0.004'),
            DeclareLaunchArgument('camera_pitch', default_value='0.1915'),
            DeclareLaunchArgument('camera_yaw', default_value='-0.035'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_mount_static_transform',
                arguments=[
                    '--x', LaunchConfiguration('camera_x'),
                    '--y', LaunchConfiguration('camera_y'),
                    '--z', LaunchConfiguration('camera_z'),
                    '--roll', LaunchConfiguration('camera_roll'),
                    '--pitch', LaunchConfiguration('camera_pitch'),
                    '--yaw', LaunchConfiguration('camera_yaw'),
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'camera_link',
                ],
                condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
                output='screen',
            ),
        ]
    )
