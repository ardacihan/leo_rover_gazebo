"""ArUco marker detection, sim and rover.

    ros2 launch leo_nav2_exploration aruco.launch.py profile:=sim
    ros2 launch leo_nav2_exploration aruco.launch.py profile:=real

The only differences between the profiles are the camera topics and the frame
convention. Gazebo's `rgbd_camera` stamps images with the *link* frame
(x forward), a RealSense stamps them with `*_color_optical_frame` (z forward),
and mixing those up rotates every detection by 90 degrees without erroring, so
`frame_is_optical` is set per profile rather than guessed.

`marker_length` is the side of the **black square**, not of the printed sheet.
`scripts/make_aruco_models.py` prints the ratio for the generated textures
(black square = 0.75 x image side).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {
    'sim': {
        'image_topic': '/leo1/camera/image',
        'camera_info_topic': '/leo1/camera/camera_info',
        'frame_is_optical': False,
        # Gazebo renders this camera at 5 Hz already.
        'rate_limit_hz': 0.0,
    },
    'real': {
        'image_topic': '/camera/camera/color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'frame_is_optical': True,
        # A RealSense streams colour at 30 Hz. Marker detection at 30 Hz on the
        # rover's computer buys nothing -- the markers are static and three
        # consecutive hits confirm one -- and it competes with SLAM for CPU.
        'rate_limit_hz': 5.0,
    },
}


def _launch_setup(context, *_args, **_kwargs):
    cfg = LaunchConfiguration
    profile = cfg('profile').perform(context)
    if profile not in PROFILES:
        raise RuntimeError(
            f'unknown profile {profile!r}; expected one of {sorted(PROFILES)}')
    preset = PROFILES[profile]

    params = {
        'use_sim_time': cfg('use_sim_time').perform(context).lower() == 'true',
        'image_topic': preset['image_topic'],
        'camera_info_topic': preset['camera_info_topic'],
        'frame_is_optical': preset['frame_is_optical'],
        'rate_limit_hz': preset['rate_limit_hz'],
        'map_frame': cfg('map_frame').perform(context),
        'dictionary': cfg('dictionary').perform(context),
        'marker_length': float(cfg('marker_length').perform(context)),
        'max_range': float(cfg('max_range').perform(context)),
        'min_hits': int(cfg('min_hits').perform(context)),
        'allowed_ids': [int(v) for v in
                        cfg('allowed_ids').perform(context).split(',') if v.strip()],
        'detection_topic': cfg('detection_topic').perform(context),
        'markers_topic': cfg('markers_topic').perform(context),
        'publish_debug_image': cfg('publish_debug_image').perform(context).lower() == 'true',
        'publish_tf': cfg('publish_tf').perform(context).lower() == 'true',
        'registry_file': cfg('registry_file').perform(context),
        'samples_file': cfg('samples_file').perform(context),
    }
    return [Node(
        package='leo_nav2_exploration',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_length', default_value='0.15'),
        DeclareLaunchArgument('max_range', default_value='6.0'),
        DeclareLaunchArgument('min_hits', default_value='3'),
        # Comma-separated ids you physically placed. Empty accepts any id.
        DeclareLaunchArgument('allowed_ids', default_value='1,2,3,4,5,6,7,8'),
        DeclareLaunchArgument('detection_topic', default_value='/aruco_detections'),
        DeclareLaunchArgument('markers_topic', default_value='/aruco_markers'),
        # The debug image is a full uncompressed RGB stream; leave it off on
        # the rover unless someone is actually looking at it.
        DeclareLaunchArgument('publish_debug_image', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        # Non-empty: periodically write the confirmed registry as JSON for
        # offline scoring against the world's ground-truth marker poses.
        DeclareLaunchArgument('registry_file', default_value=''),
        # Non-empty: per-detection CSV for marker-length verification.
        DeclareLaunchArgument('samples_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
