# Leo Nav2 Exploration Overlay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a copy-in ROS 2 Humble overlay bundle that provides robust SLAM, polygon-footprint Nav2 navigation, doorway simulation regression, optional depth-camera voxel obstacles, frontier exploration, calibration tools, and preflight checks.

**Architecture:** The package starts SLAM Toolbox and a non-composed Nav2 graph with Smac State Lattice and DWB. Commands flow from controller to velocity smoother to a custom fail-closed guard to Nav2 Collision Monitor, which exclusively publishes the final robot command. Separate simulator and real profiles keep topic/frame ownership explicit and never replace the existing robot-description, odometry, or static-TF publishers.

**Tech Stack:** ROS 2 Humble, Python 3, `ament_python`, Nav2, SLAM Toolbox, Smac State Lattice, DWB, Nav2 VoxelLayer, Nav2 Collision Monitor, `rclpy`, NumPy, PyYAML, pytest, Gazebo/ros_gz, `frontier_exploration_ros2` v1.6.1.

## Global Constraints

- Create a standalone overlay; do not modify `leo_rover_real_bringup`, `leo_rover_gazebo`, or the current reactive explorer.
- Support exactly two baseline profiles: `sim_leo1` and `real_root`.
- LiDAR is the only SLAM source; PointCloud2 is optional and local-costmap-only.
- Use a polygon footprint with independent front/rear/left/right extents and 0.01 m starting padding.
- Collision Monitor is the only final `/cmd_vel` or `/leo1/cmd_vel` publisher.
- Exploration starts disabled until a manual doorway goal succeeds.
- No package may publish robot static transforms or `odom -> base`.
- ROS 2/Gazebo runtime claims require verification on the user's Humble system; local validation is static and unit-level only.

---

### Task 1: Scaffold and pure geometry contracts

**Files:**
- Create: `src/leo_nav2_exploration/package.xml`
- Create: `src/leo_nav2_exploration/setup.py`
- Create: `src/leo_nav2_exploration/setup.cfg`
- Create: `src/leo_nav2_exploration/resource/leo_nav2_exploration`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/__init__.py`
- Test: `src/leo_nav2_exploration/test/test_geometry.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/geometry.py`

**Interfaces:**
- Produces: `FootprintExtents`, `footprint_points()`, `footprint_yaml_string()`, `circumscribed_radius()`, and `doorway_margin()`.

- [ ] Write geometry tests for centered/asymmetric footprints, invalid dimensions, radius, and doorway margin.
- [ ] Run the test and confirm import/feature failures.
- [ ] Implement the minimal geometry module.
- [ ] Run geometry tests and confirm they pass.

### Task 2: Velocity-guard logic and ROS node

**Files:**
- Test: `src/leo_nav2_exploration/test/test_velocity_guard_logic.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/velocity_guard_logic.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/velocity_guard_node.py`

**Interfaces:**
- Consumes: command, scan, odometry, and optional battery timestamps and values.
- Produces: `GuardConfig`, `GuardState`, `GuardDecision`, `evaluate_guard()`, and console script `velocity_guard`.

- [ ] Write tests for stale command, stale scan, stale odometry, optional battery behavior, minimum battery, clamping, and valid pass-through.
- [ ] Run tests and confirm they fail because the logic is absent.
- [ ] Implement pure fail-closed guard logic.
- [ ] Run tests and confirm they pass.
- [ ] Implement the thin `rclpy` wrapper with no duplicated decision logic.

### Task 3: Calibration mathematics and tools

**Files:**
- Test: `src/leo_nav2_exploration/test/test_calibration_math.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/calibration_math.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/footprint_tool.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/lidar_board_calibration.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/camera_floor_calibration.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/odom_calibration.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/tf_snapshot.py`

**Interfaces:**
- Produces: robust sector statistics, plane fit and orientation estimates, linear/angular odometry scale factors, footprint CLI, and observation-only ROS calibration nodes.

- [ ] Write tests for robust median sectors, plane fitting, camera roll/pitch extraction, angle unwrapping, and odometry scales.
- [ ] Run tests and confirm missing-feature failures.
- [ ] Implement pure calibration mathematics.
- [ ] Run tests and confirm they pass.
- [ ] Implement ROS/CLI wrappers that print recommendations and never publish TF.

### Task 4: Nav2, SLAM, Collision Monitor, exploration, and BT configuration

**Files:**
- Test: `src/leo_nav2_exploration/test/test_config_contracts.py`
- Create: `src/leo_nav2_exploration/config/sim/nav2.yaml`
- Create: `src/leo_nav2_exploration/config/sim/slam.yaml`
- Create: `src/leo_nav2_exploration/config/sim/collision_monitor.yaml`
- Create: `src/leo_nav2_exploration/config/sim/velocity_guard.yaml`
- Create: `src/leo_nav2_exploration/config/sim/frontier.yaml`
- Create: `src/leo_nav2_exploration/config/real/nav2.yaml`
- Create: `src/leo_nav2_exploration/config/real/slam.yaml`
- Create: `src/leo_nav2_exploration/config/real/collision_monitor.yaml`
- Create: `src/leo_nav2_exploration/config/real/velocity_guard.yaml`
- Create: `src/leo_nav2_exploration/config/real/frontier.yaml`
- Create: `src/leo_nav2_exploration/behavior_trees/navigate_to_pose_doorway_recovery.xml`

**Interfaces:**
- Produces: matching 0.05 m global State-Lattice planning configs, 0.025 m local costmaps, full-footprint DWB checks, optional local PointCloud2 VoxelLayer, conservative SLAM loop closure, and backup-first recovery.

- [ ] Write static contract tests that load both profiles and assert footprint, plugin, topic, resolution, command chain, sensor-source, and exploration cold-idle invariants.
- [ ] Run tests and confirm missing-file failures.
- [ ] Create the parameter and BT files.
- [ ] Run contract tests and correct all failures.

### Task 5: Launch graph and preflight ownership audit

**Files:**
- Test: `src/leo_nav2_exploration/test/test_launch_support.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/launch_support.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/preflight_check.py`
- Create: `src/leo_nav2_exploration/launch/navigation_overlay.launch.py`
- Create: `src/leo_nav2_exploration/launch/sim_navigation.launch.py`
- Create: `src/leo_nav2_exploration/launch/real_navigation.launch.py`
- Create: `src/leo_nav2_exploration/launch/frontier_exploration.launch.py`

**Interfaces:**
- Produces: runtime lattice-file resolution, profile path resolution, one explicit non-composed Nav2 graph, one lifecycle manager, guard/Collision-Monitor chain, and a read-only ownership/freshness preflight CLI.

- [ ] Write launch-support tests for profile validation, package-share paths, and lattice-path construction.
- [ ] Run tests and confirm missing-feature failures.
- [ ] Implement launch support and verify tests pass.
- [ ] Implement launch files and preflight node.
- [ ] Compile launch sources and inspect the remapping chain statically.

### Task 6: Doorway simulation regression and goal clients

**Files:**
- Test: `src/leo_nav2_exploration/test/test_regression_scenarios.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/navigate_goal.py`
- Create: `src/leo_nav2_exploration/leo_nav2_exploration/doorway_regression.py`
- Create: `src/leo_nav2_exploration/config/sim/doorway_goals.yaml`
- Create: `src/leo_nav2_exploration/models/doorway_fixture/model.config`
- Create: `src/leo_nav2_exploration/models/doorway_fixture/model.sdf`
- Create: `src/leo_nav2_exploration/launch/sim_doorway_regression.launch.py`

**Interfaces:**
- Produces: `NavigateToPose` CLI, sequential regression runner with per-goal timeout and exit status, and a 0.78 m clear-width vertical-plane fixture in the existing empty Gazebo world.

- [ ] Write scenario tests for valid goal schema, alternating doorway sides, clear-width consistency, and timeout values.
- [ ] Run tests and confirm missing-file failures.
- [ ] Create fixture, scenarios, action client, and regression runner.
- [ ] Run tests and Python compilation.

### Task 7: Operator scripts, dependency pinning, and documentation

**Files:**
- Create: `dependencies.repos`
- Create: `scripts/install_dependencies.sh`
- Create: `scripts/build_overlay.sh`
- Create: `scripts/run_sim_doorway.sh`
- Create: `scripts/run_sim_navigation.sh`
- Create: `scripts/run_real_navigation.sh`
- Create: `scripts/run_preflight.sh`
- Create: `scripts/send_goal.sh`
- Create: `scripts/start_exploration.sh`
- Create: `scripts/stop_exploration.sh`
- Create: `scripts/explore_and_save.sh`
- Create: `scripts/save_map.sh`
- Create: `scripts/record_debug_bag.sh`
- Create: `docs/README.md`
- Create: `docs/INTEGRATION_FOR_CLAUDE_CODE.md`
- Create: `docs/CALIBRATION.md`
- Create: `docs/SIMULATION_TEST_PLAN.md`

**Interfaces:**
- Produces: copy/build/run workflow, pinned Humble exploration dependency, exact calibration procedure, and simulator acceptance criteria.

- [ ] Add scripts with strict shell error handling and explicit profile/topic defaults.
- [ ] Run `bash -n` on every shell script.
- [ ] Write operator and Claude Code integration documentation with exact commands and measurement instructions.

### Task 8: Bundle validation and archive

**Files:**
- Create: `validate_bundle.sh`
- Create: `MANIFEST.sha256`
- Create: archive `leo_nav2_exploration_bundle.zip`

**Interfaces:**
- Produces: reproducible static validation and integrity manifest.

- [ ] Run all pytest tests.
- [ ] Compile all Python and launch sources.
- [ ] Parse all YAML and XML files.
- [ ] Validate package metadata, script syntax, executable bits, command chain, and absence of TF publishers.
- [ ] Generate SHA-256 manifest excluding the manifest itself.
- [ ] Run `validate_bundle.sh` from a clean extraction.
- [ ] Create the ZIP and verify it with `unzip -t`.
