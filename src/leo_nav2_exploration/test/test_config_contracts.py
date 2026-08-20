from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
PROFILES = {
    "sim": {
        "base": "leo1/base_link",
        "odom": "leo1/odom",
        "raw_scan": "/leo1/scan",
        "scan": "/leo1/scan_filtered",
        "cloud": "/leo1/camera/points",
        "odom_topic": "/leo1/odom",
        "nav": "/leo1/cmd_vel_nav",
        "smoothed": "/leo1/cmd_vel_smoothed",
        "guarded": "/leo1/cmd_vel_guarded",
        "final": "/leo1/cmd_vel",
    },
    "real": {
        "base": "base_footprint",
        "odom": "odom",
        "raw_scan": "/scan",
        "scan": "/scan_filtered",
        "cloud": "/camera_points_filtered",
        "odom_topic": "/wheel_odom",
        "nav": "/cmd_vel_nav",
        "smoothed": "/cmd_vel_smoothed",
        "guarded": "/cmd_vel_guarded",
        "final": "/cmd_vel",
    },
}


def load(profile: str, name: str):
    with (PACKAGE / "config" / profile / f"{name}.yaml").open() as handle:
        return yaml.safe_load(handle)


def parse_footprint(value: str):
    result = yaml.safe_load(value)
    assert isinstance(result, list)
    return result


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_lidar_filter_removes_only_returns_inside_robot_box(profile):
    expected = PROFILES[profile]
    filt = load(profile, "scan_filter")["scan_to_scan_filter_chain"]["ros__parameters"]
    box = filt["filter1"]
    assert box["type"] == "laser_filters/LaserScanBoxFilter"
    params = box["params"]
    assert params["box_frame"] == expected["base"]
    assert params["invert"] is False
    assert params["min_x"] <= -0.22 and params["max_x"] >= 0.22
    assert params["min_y"] <= -0.22 and params["max_y"] >= 0.22
    assert params["min_z"] < 0.0 < params["max_z"]
    assert params["max_x"] - params["min_x"] <= 0.52
    assert params["max_y"] - params["min_y"] <= 0.52


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_nav2_uses_polygon_footprint_and_state_lattice(profile):
    nav = load(profile, "nav2")
    local = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    assert "robot_radius" not in local
    assert "robot_radius" not in global_
    assert parse_footprint(local["footprint"]) == parse_footprint(global_["footprint"])
    assert parse_footprint(local["footprint"]) == [
        [0.21, 0.21], [0.21, -0.21], [-0.21, -0.21], [-0.21, 0.21]
    ]
    assert local["footprint_padding"] == pytest.approx(0.01)
    assert global_["footprint_padding"] == pytest.approx(0.01)
    planner = nav["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert planner["lattice_filepath"] == "__LATTICE_FILE__"
    assert planner["rotation_penalty"] >= 7.0
    assert planner["allow_reverse_expansion"] is False


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_costmaps_preserve_doorway_resolution_and_center_bias(profile):
    nav = load(profile, "nav2")
    local = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["resolution"] == pytest.approx(0.025)
    assert global_["resolution"] == pytest.approx(0.05)
    assert global_["inflation_layer"]["inflation_radius"] >= 0.45
    assert local["inflation_layer"]["inflation_radius"] >= 0.45
    assert global_["inflation_layer"]["cost_scaling_factor"] >= 3.0


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_dwb_checks_full_footprint_and_uses_slow_hardware_like_limits(profile):
    nav = load(profile, "nav2")
    controller = nav["controller_server"]["ros__parameters"]
    shim = controller["FollowPath"]
    assert shim["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert 0.25 <= shim["angular_dist_threshold"] <= 0.60
    assert 0.05 <= shim["angular_disengage_threshold"] < shim["angular_dist_threshold"]
    assert shim["closed_loop"] is True
    assert shim["rotate_to_heading_angular_vel"] <= 0.30
    assert shim["max_angular_accel"] <= 0.70
    assert shim["simulate_ahead_time"] >= 1.0
    # ROS 2 Humble Rotation Shim loads the primary controller by type string;
    # the DWB parameters remain in the same FollowPath namespace.
    assert shim["primary_controller"] == "dwb_core::DWBLocalPlanner"
    assert "ObstacleFootprint" in shim["critics"]
    assert "BaseObstacle" not in shim["critics"]
    assert shim["ObstacleFootprint"]["plugin"] == "dwb_critics::ObstacleFootprintCritic"
    assert shim["max_vel_x"] <= 0.12
    assert shim["max_vel_theta"] <= 0.40
    assert shim["acc_lim_x"] <= 0.30
    assert shim["acc_lim_theta"] <= 0.70
    assert controller["progress_checker"]["required_movement_radius"] <= 0.12


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_sensor_topics_and_frames_match_profile(profile):
    expected = PROFILES[profile]
    nav = load(profile, "nav2")
    local = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    bt = nav["bt_navigator"]["ros__parameters"]
    assert local["robot_base_frame"] == expected["base"]
    assert global_["robot_base_frame"] == expected["base"]
    assert local["global_frame"] == expected["odom"]
    assert bt["robot_base_frame"] == expected["base"]
    assert bt["odom_topic"] == expected["odom_topic"]
    assert local["obstacle_layer"]["scan"]["topic"] == expected["scan"]
    assert global_["obstacle_layer"]["scan"]["topic"] == expected["scan"]
    # The camera must live in its own layer: sharing the scan's layer lets
    # lidar raytrace clearing erase camera marks for obstacles below the
    # lidar plane (found on drive_2026-08-20).
    assert "camera" not in local["obstacle_layer"]["observation_sources"].split()
    camera = local["camera_obstacle_layer"]["camera"]
    assert local["camera_obstacle_layer"]["observation_sources"] == "camera"
    assert camera["topic"] == expected["cloud"]
    assert camera["data_type"] == "PointCloud2"
    assert camera["marking"] is True
    assert camera["clearing"] is True
    assert camera["min_obstacle_height"] <= 0.06  # shoes and pi-pucks
    assert "camera_obstacle_layer" in local["plugins"]
    assert local["plugins"].index("camera_obstacle_layer") < local["plugins"].index("inflation_layer")


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_slam_is_lidar_only_and_robustified(profile):
    expected = PROFILES[profile]
    slam = load(profile, "slam")["slam_toolbox"]["ros__parameters"]
    assert slam["scan_topic"] == expected["scan"]
    assert slam["base_frame"] == expected["base"]
    assert slam["odom_frame"] == expected["odom"]
    assert slam["resolution"] == pytest.approx(0.05)
    assert slam["scan_queue_size"] == 1
    assert slam["ceres_loss_function"] == "HuberLoss"
    assert slam["enable_interactive_mode"] is False
    assert slam["do_loop_closing"] is True
    assert slam["loop_match_minimum_chain_size"] >= 5
    assert "camera" not in str(slam).lower()


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_guard_and_collision_monitor_form_single_final_command_chain(profile):
    expected = PROFILES[profile]
    guard = load(profile, "velocity_guard")["velocity_guard"]["ros__parameters"]
    monitor = load(profile, "collision_monitor")["collision_monitor"]["ros__parameters"]
    assert guard["input_topic"] == expected["smoothed"]
    assert guard["output_topic"] == expected["guarded"]
    assert guard["scan_topic"] == expected["scan"]
    assert guard["odom_topic"] == expected["odom_topic"]
    assert guard["require_camera"] is False
    assert guard["minimum_valid_scan_points"] >= 20
    assert monitor["cmd_vel_in_topic"] == expected["guarded"]
    assert monitor["cmd_vel_out_topic"] == expected["final"]
    assert monitor["base_frame_id"] == expected["base"]
    assert monitor["odom_frame_id"] == expected["odom"]
    assert monitor["polygons"][0] == "StopZone"
    stop_points = monitor["StopZone"]["points"]
    assert isinstance(stop_points, list) and len(stop_points) == 8
    assert max(abs(value) for value in stop_points) <= 0.26
    assert max(abs(value) for value in stop_points) >= 0.25
    assert monitor["StopZone"]["action_type"] == "stop"
    assert monitor["FootprintApproach"]["type"] == "polygon"
    assert monitor["FootprintApproach"]["footprint_topic"] == "/local_costmap/published_footprint"
    assert monitor["scan"]["topic"] == expected["scan"]
    assert monitor["observation_sources"] == ["scan"]
    # ROS 2 Humble uses one global source_timeout and has no state_topic parameter.
    assert "source_timeout" not in monitor["scan"]
    assert "state_topic" not in monitor


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_frontier_exploration_starts_idle_and_filters_micro_frontiers(profile):
    frontier = load(profile, "frontier")["frontier_explorer"]["ros__parameters"]
    assert frontier["autostart"] is False
    assert frontier["control_service_enabled"] is True
    assert frontier["completion_event_enabled"] is True
    assert frontier["frontier_map_optimization_enabled"] is True
    assert frontier["goal_preemption_enabled"] is True
    assert frontier["min_frontier_size_cells"] >= 8
    assert frontier["frontier_candidate_min_goal_distance_m"] >= 0.65
    assert frontier["frontier_selection_min_distance"] >= 0.65
    assert frontier["frontier_suppression_enabled"] is True
    assert frontier["return_to_start_on_complete"] is False


def test_recovery_tree_backs_up_before_attempting_spin():
    root = ET.parse(PACKAGE / "behavior_trees" / "navigate_to_pose_doorway_recovery.xml").getroot()
    round_robin = root.find(".//RoundRobin")
    tags = [child.tag for child in list(round_robin)]
    assert tags.index("BackUp") < tags.index("Spin")
    backup = round_robin.find("BackUp")
    spin = round_robin.find("Spin")
    assert float(backup.attrib["backup_speed"]) <= 0.05
    assert float(spin.attrib["spin_dist"]) <= 0.8
