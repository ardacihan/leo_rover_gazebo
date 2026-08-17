from pathlib import Path

import pytest
import yaml

from leo_nav2_exploration.launch_support import (
    materialize_parameter_file,
    resolve_profile_paths,
    sample_lattice_path,
)


def test_resolve_profile_paths_selects_all_profile_files(tmp_path):
    for profile in ("sim", "real"):
        config = tmp_path / "config" / profile
        config.mkdir(parents=True)
        for name in ("nav2", "scan_filter", "slam", "collision_monitor", "velocity_guard", "frontier"):
            (config / f"{name}.yaml").write_text("node: {ros__parameters: {}}\n")
    bt = tmp_path / "behavior_trees"
    bt.mkdir()
    (bt / "navigate_to_pose_doorway_recovery.xml").write_text("<root/>")

    paths = resolve_profile_paths(tmp_path, "sim_leo1")
    assert paths.profile_directory.name == "sim"
    assert paths.nav2.name == "nav2.yaml"
    assert paths.behavior_tree.name == "navigate_to_pose_doorway_recovery.xml"


def test_resolve_profile_paths_rejects_unknown_profile(tmp_path):
    with pytest.raises(ValueError, match="profile"):
        resolve_profile_paths(tmp_path, "wrong")


def test_sample_lattice_path_uses_humble_installed_sample_location(tmp_path):
    expected = (
        tmp_path
        / "sample_primitives"
        / "5cm_resolution"
        / "0.5m_turning_radius"
        / "diff"
        / "output.json"
    )
    expected.parent.mkdir(parents=True)
    expected.write_text("{}")
    assert sample_lattice_path(tmp_path) == expected


def test_materialize_parameter_file_replaces_exact_placeholders(tmp_path):
    source = tmp_path / "source.yaml"
    source.write_text(
        "planner_server:\n"
        "  ros__parameters:\n"
        "    GridBased:\n"
        "      lattice_filepath: __LATTICE_FILE__\n"
        "bt_navigator:\n"
        "  ros__parameters:\n"
        "    default_nav_to_pose_bt_xml: __BT_XML__\n"
    )
    output = materialize_parameter_file(
        source,
        {"__LATTICE_FILE__": "/a/lattice.json", "__BT_XML__": "/a/tree.xml"},
        output_directory=tmp_path,
    )
    data = yaml.safe_load(output.read_text())
    assert data["planner_server"]["ros__parameters"]["GridBased"]["lattice_filepath"] == "/a/lattice.json"
    assert data["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] == "/a/tree.xml"
    assert output != source


def test_materialize_parameter_file_applies_nested_path_overrides(tmp_path):
    source = tmp_path / "source.yaml"
    source.write_text(
        "local_costmap:\n"
        "  local_costmap:\n"
        "    ros__parameters:\n"
        "      voxel_layer:\n"
        "        enabled: true\n"
    )
    output = materialize_parameter_file(
        source,
        {},
        output_directory=tmp_path,
        path_overrides={
            ("local_costmap", "local_costmap", "ros__parameters", "voxel_layer", "enabled"): False
        },
    )
    data = yaml.safe_load(output.read_text())
    assert data["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]["enabled"] is False
