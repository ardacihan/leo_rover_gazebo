from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


def _source(name: str) -> str:
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


def test_all_operator_launch_files_exist():
    expected = {
        "navigation_overlay.launch.py",
        "sim_navigation.launch.py",
        "real_navigation.launch.py",
        "frontier_exploration.launch.py",
        "sim_doorway_regression.launch.py",
    }
    assert expected <= {path.name for path in LAUNCH_DIR.glob("*.launch.py")}


def test_navigation_overlay_owns_only_navigation_and_safety_nodes():
    source = _source("navigation_overlay.launch.py")
    forbidden = (
        "static_transform_publisher",
        "robot_state_publisher",
        "wheel_odom",
        "odom_tf_broadcaster",
    )
    for token in forbidden:
        assert token not in source

    for executable in (
        "scan_to_scan_filter_chain",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "velocity_guard",
        "collision_monitor",
        "lifecycle_manager",
    ):
        assert executable in source


def test_navigation_overlay_has_explicit_guarded_command_chain():
    source = _source("navigation_overlay.launch.py")
    assert "cmd_vel_nav" in source
    assert "cmd_vel_smoothed" in source
    assert "cmd_vel_guarded" in source
    assert "lifecycle_manager_navigation" in source
    assert "lifecycle_manager_collision_monitor" in source
    assert "('cmd_vel', topics.cmd_vel_nav)" in source
    assert "('cmd_vel_smoothed', topics.cmd_vel_smoothed)" in source


def test_real_profile_keeps_voxel_layer_opt_in():
    source = _source("real_navigation.launch.py")
    assert "profile': 'real_root'" in source
    assert "default_value='false'" in source
    assert "enable_voxel" in source


def test_sim_profile_enables_voxel_layer_by_default():
    source = _source("sim_navigation.launch.py")
    assert "profile': 'sim_leo1'" in source
    assert "default_value='true'" in source
    assert "enable_voxel" in source


def test_doorway_regression_launch_is_manual_by_default_and_spawns_fixture():
    source = _source("sim_doorway_regression.launch.py")
    assert "leo_rover_gazebo" in source
    assert "doorway_fixture" in source
    assert "run_regression" in source
    assert "default_value='false'" in source
    assert "doorway_regression" in source


def test_navigation_overlay_filters_raw_lidar_before_slam_navigation_and_safety():
    source = _source("navigation_overlay.launch.py")
    assert "package='laser_filters'" in source
    assert "executable='scan_to_scan_filter_chain'" in source
    assert "paths.scan_filter" in source
    assert "('scan', scan_topics.raw)" in source
    assert "('scan_filtered', scan_topics.filtered)" in source


def test_namespaced_real_aruco_uses_that_rovers_camera_and_map():
    mapping = _source("real_mapping.launch.py")
    aruco = _source("aruco.launch.py")

    assert "f'{p}/camera/camera/color/image_raw'" in mapping
    assert "f'{p}/camera/camera/color/camera_info'" in mapping
    assert "f'{p}/tag_detections'" in mapping
    assert "f'{p}/aruco/debug_image'" in mapping
    assert "DeclareLaunchArgument('aruco_max_range', default_value='4.5')" in mapping
    assert "image_topic or _ns(preset['image_topic'])" in aruco
    assert "camera_info_topic or _ns(preset['camera_info_topic'])" in aruco
    assert "map_frame = f'{ns}/map'" in aruco
