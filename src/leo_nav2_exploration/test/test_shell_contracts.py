from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[3]
SCRIPTS = BUNDLE / "scripts"


def test_frontier_control_scripts_use_ros2_run_not_path_lookup():
    for name in ("start_exploration.sh", "stop_exploration.sh", "explore_and_save.sh"):
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert "ros2 run frontier_exploration_ros2 frontier_exploration_ctl" in source
        assert "require_command frontier_exploration_ctl" not in source


def test_required_operator_documents_exist():
    docs = BUNDLE / "docs"
    for name in (
        "README.md",
        "INTEGRATION_FOR_CLAUDE_CODE.md",
        "CALIBRATION.md",
        "SIMULATION_TEST_PLAN.md",
    ):
        path = docs / name
        assert path.is_file(), f"missing {path}"
        assert path.stat().st_size > 1000, f"document too small: {path}"


def test_debug_and_calibration_scripts_distinguish_raw_and_filtered_lidar():
    record = (SCRIPTS / "record_debug_bag.sh").read_text(encoding="utf-8")
    calibration = (SCRIPTS / "calibration_report.sh").read_text(encoding="utf-8")
    assert "/leo1/scan_filtered" in record and "/scan_filtered" in record
    assert "RAW_SCAN" in record and "FILTERED_SCAN" in record
    assert "/leo1/scan_filtered" in calibration and "/scan_filtered" in calibration
    assert "raw scan" in calibration.lower() and "filtered scan" in calibration.lower()


def test_bundle_validator_runs_tests_parsers_and_shell_checks():
    validator = BUNDLE / "validate_bundle.sh"
    assert validator.is_file()
    source = validator.read_text(encoding="utf-8")
    for token in ("pytest", "compileall", "yaml.safe_load", "ElementTree", "bash -n", "MANIFEST.sha256"):
        assert token in source
