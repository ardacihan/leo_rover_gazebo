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
        # `{ns}` is substituted with the robot_ns argument, so two rovers each
        # get a detector without a second launch file.
        'image_topic': '/{ns}/camera/image',
        'camera_info_topic': '/{ns}/camera/camera_info',
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

    # Empty robot_ns keeps the single-robot topic names the rover uses.
    ns = cfg('robot_ns').perform(context).strip('/')
    def _ns(value):
        return value.format(ns=ns) if ns else value.replace('/{ns}', '')

    map_frame = cfg('map_frame').perform(context)
    if ns and map_frame == 'map':
        # Each rover's SLAM publishes in its own map frame, and the detector
        # must resolve tag poses there or the aligner compares landmarks
        # expressed in two different frames as if they shared one.
        map_frame = f'{ns}/map'

    # The presets are safe single-rover defaults.  A physical two-rover graph
    # must be able to point each detector at its own namespaced RealSense;
    # otherwise both namespaced detector nodes still subscribe to the same
    # absolute /camera/... topic and one rover's landmarks are silently
    # attributed to the other rover.
    image_topic = cfg('image_topic').perform(context).strip()
    camera_info_topic = cfg('camera_info_topic').perform(context).strip()
    camera_frame_override = cfg('camera_frame_override').perform(context).strip()

    params = {
        'use_sim_time': cfg('use_sim_time').perform(context).lower() == 'true',
        'image_topic': image_topic or _ns(preset['image_topic']),
        'camera_info_topic': camera_info_topic or _ns(preset['camera_info_topic']),
        'frame_is_optical': preset['frame_is_optical'],
        'rate_limit_hz': preset['rate_limit_hz'],
        'map_frame': map_frame,
        'dictionary': cfg('dictionary').perform(context),
        'marker_length': float(cfg('marker_length').perform(context)),
        'max_range': float(cfg('max_range').perform(context)),
        'min_hits': int(cfg('min_hits').perform(context)),
        'allowed_ids': [int(v) for v in
                        cfg('allowed_ids').perform(context).split(',') if v.strip()],
        'detection_topic': _ns(cfg('detection_topic').perform(context)),
        'markers_topic': _ns(cfg('markers_topic').perform(context)),
        'tag_frame_prefix': f'{ns}/' if ns else '',
        'camera_frame_override': camera_frame_override,
        'publish_debug_image': cfg('publish_debug_image').perform(context).lower() == 'true',
        'debug_image_topic': _ns(cfg('debug_image_topic').perform(context)),
        'publish_tf': cfg('publish_tf').perform(context).lower() == 'true',
        'registry_file': cfg('registry_file').perform(context),
        'samples_file': cfg('samples_file').perform(context),
    }
    return [Node(
        package='leo_nav2_exploration',
        executable='aruco_detector',
        name='aruco_detector',
        namespace=ns or None,
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='real'),
        # e.g. 'leo1'. Namespaces the node and substitutes {ns} in
        # the sim profile's topics; empty keeps the rover's names.
        DeclareLaunchArgument('robot_ns', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Leave empty for the selected profile's single-rover defaults.  Set
        # these explicitly for namespaced real cameras.
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value=''),
        DeclareLaunchArgument('camera_frame_override', default_value=''),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50'),
        # Side of the black square. The sim plates are 0.20 m edge to edge
        # (their textures carry no quiet zone -- the world geometry does).
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
        # Absolute, so two rovers would collide on one topic unless it
        # carries {ns} (substituted with robot_ns).
        DeclareLaunchArgument('debug_image_topic',
                              default_value='/aruco/debug_image'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        # Non-empty: periodically write the confirmed registry as JSON for
        # offline scoring against the world's ground-truth marker poses.
        DeclareLaunchArgument('registry_file', default_value=''),
        # Non-empty: per-detection CSV for marker-length verification.
        DeclareLaunchArgument('samples_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
