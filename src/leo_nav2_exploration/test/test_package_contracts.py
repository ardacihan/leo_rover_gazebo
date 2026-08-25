from pathlib import Path
import ast
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_plugins_and_launch_dependencies():
    root = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text.strip() for element in root.findall("exec_depend")}
    required = {
        "launch",
        "launch_ros",
        "laser_filters",
        "nav2_costmap_2d",
        "nav2_rotation_shim_controller",
        "dwb_core",
        "dwb_critics",
        "nav2_map_server",
        "frontier_exploration_ros2",
        "ros_gz_sim",
    }
    assert required <= dependencies


def test_setup_installs_launch_config_bt_and_fixture_assets():
    source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert "launch/*.launch.py" in source
    assert "config/sim/*.yaml" in source
    assert "config/real/*.yaml" in source
    assert "behavior_trees/*.xml" in source
    assert "models/doorway_fixture/*" in source
    assert any(isinstance(node, ast.Call) and getattr(node.func, "id", "") == "setup" for node in ast.walk(tree))
