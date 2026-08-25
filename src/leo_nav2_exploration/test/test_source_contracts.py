from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
MODULES = PACKAGE / "leo_nav2_exploration"


def test_camera_floor_calibration_has_one_collector_and_constrains_floor_normal():
    source = (MODULES / "camera_floor_calibration.py").read_text(encoding="utf-8")
    assert source.count("class CameraFloorCollector") == 1
    assert '"--max-floor-tilt-deg"' in source
    assert "expected_normal=np.array([0.0, -1.0, 0.0])" in source
    assert "maximum_normal_angle=math.radians(cli.max_floor_tilt_deg)" in source


def test_velocity_guard_reports_scan_quality_separately_from_freshness():
    source = (MODULES / "velocity_guard_node.py").read_text(encoding="utf-8")
    assert 'declare_parameter("minimum_valid_scan_points", 30)' in source
    assert 'declare_parameter("front_stop_distance", 0.0)' in source
    assert "scan_valid_points=self._scan_valid_points" in source
    assert "front_min_range=self._front_min_range" in source


def test_preflight_requires_both_raw_and_filtered_scan_publishers():
    source = (MODULES / "preflight_check.py").read_text(encoding="utf-8")
    assert 'profile["raw_scan"]' in source
    assert 'profile["scan"]' in source
    assert "raw scan publishers" in source
    assert "filtered scan publishers" in source


def test_firmware_bridge_applies_udp_only_before_rclpy():
    source = (MODULES / "firmware_bridge.py").read_text(encoding="utf-8")
    side = source.split("def _firmware_side", 1)[1]
    apply_at = side.find("apply_udp_only_transport()")
    rclpy_at = side.find("import rclpy")
    assert apply_at != -1
    assert rclpy_at != -1
    assert apply_at < rclpy_at
    assert 'os.environ.get("LEO_FIRMWARE_DOMAIN", "2")' in source
    assert 'os.environ.get("LEO_NAV_DOMAIN", "22")' in source
    assert '"/wheel_odom"' in source
    assert '"/rob_2"' not in source.split("Rover 4 must set", 1)[0]


def test_isolated_navigation_does_not_replace_localhost_fastdds():
    source = (PACKAGE / "launch" / "isolated_real_navigation.launch.py").read_text(
        encoding="utf-8"
    )
    assert "SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')" in source
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" not in source.split('"""', 2)[-1]


def test_isolated_firmware_is_domain_two_udp_only():
    source = (PACKAGE / "launch" / "isolated_firmware.launch.py").read_text(encoding="utf-8")
    assert "'ROS_DOMAIN_ID', '2'" in source
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" in source
    assert "leo_bringup.launch.xml" in source
    assert "jetson-04" in source

