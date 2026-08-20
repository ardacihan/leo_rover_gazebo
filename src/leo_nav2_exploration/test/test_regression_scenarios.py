from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def load_scenarios():
    return yaml.safe_load((PACKAGE / "config" / "sim" / "doorway_goals.yaml").read_text())


def test_doorway_fixture_clear_width_matches_scenario_metadata():
    scenarios = load_scenarios()
    root = ET.parse(PACKAGE / "models" / "doorway_fixture" / "model.sdf").getroot()
    links = {link.attrib["name"]: link for link in root.findall(".//link")}
    lower = links["door_wall_lower"]
    upper = links["door_wall_upper"]

    def center_y(link):
        return float(link.find("pose").text.split()[1])

    def size_y(link):
        return float(link.find("collision/geometry/box/size").text.split()[1])

    lower_inner = center_y(lower) + size_y(lower) / 2.0
    upper_inner = center_y(upper) - size_y(upper) / 2.0
    clear_width = upper_inner - lower_inner
    assert clear_width == pytest.approx(scenarios["fixture"]["door_clear_width"], abs=1e-6)
    assert clear_width == pytest.approx(0.78)


def test_padded_robot_has_at_least_ten_centimetres_total_door_margin():
    scenarios = load_scenarios()
    fixture = scenarios["fixture"]
    padded_width = fixture["robot_physical_width"] + 2 * fixture["footprint_padding"]
    assert fixture["door_clear_width"] - padded_width >= 0.10


def test_goals_are_relative_to_start_and_alternate_across_doorway():
    scenarios = load_scenarios()
    assert scenarios["goal_coordinates"] == "relative_to_start"
    assert scenarios["robot_base_frame"] == "leo1/base_link"
    goals = scenarios["goals"]
    assert len(goals) >= 6
    # Start-side targets are x=0; far-side targets are x=3 m from the start pose.
    sides = [1 if goal["x"] > 1.5 else -1 for goal in goals]
    assert all(a != b for a, b in zip(sides, sides[1:]))
    assert all(abs(goal["y"]) <= 0.10 for goal in goals)
    assert all(goal["timeout_sec"] >= 90.0 for goal in goals)


def test_scenario_has_repeatability_thresholds():
    acceptance = load_scenarios()["acceptance"]
    assert acceptance["required_successes"] >= 6
    assert acceptance["max_failures"] <= 1
    assert acceptance["maximum_contact_count"] == 0
