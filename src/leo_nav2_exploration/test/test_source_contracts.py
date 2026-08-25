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
    assert "scan_valid_points=self._scan_valid_points" in source


def test_preflight_requires_both_raw_and_filtered_scan_publishers():
    source = (MODULES / "preflight_check.py").read_text(encoding="utf-8")
    assert 'profile["raw_scan"]' in source
    assert 'profile["scan"]' in source
    assert "raw scan publishers" in source
    assert "filtered scan publishers" in source
