# Leo Nav2 Exploration Overlay

This bundle adds a standalone ROS 2 Humble navigation package named `leo_nav2_exploration`. It does not overwrite `leo_rover_gazebo`, `leo_rover_real_bringup`, the existing reactive explorer, robot-description files, wheel odometry, or static transforms.

The purpose is to replace random forward/turn exploration with a conventional map-aware stack that can deliberately align with and traverse a doorway:

```text
raw 2D LiDAR
    -> footprint-aware LaserScan self filter
    -> SLAM Toolbox (LiDAR only)
    -> Nav2 global and local costmaps
    -> Smac State Lattice global planner
    -> Rotation Shim + DWB footprint-aware controller
    -> Nav2 velocity smoother
    -> fail-closed velocity guard
    -> Nav2 Collision Monitor
    -> final robot cmd_vel
```

The depth camera is optional. When enabled, its `PointCloud2` data is used only by the local VoxelLayer. It does not gate motion, does not replace LiDAR SLAM, and cannot declare a LiDAR obstacle free merely because the camera failed to see it.

## Profiles

| Profile | Raw scan | Filtered scan | Odometry | Base frame | Camera cloud | Final command |
|---|---|---|---|---|---|---|
| `sim_leo1` | `/leo1/scan` | `/leo1/scan_filtered` | `/leo1/odom` | `leo1/base_link` | `/leo1/camera/points` | `/leo1/cmd_vel` |
| `real_root` | `/scan` | `/scan_filtered` | `/wheel_odom` | `base_footprint` | `/camera/camera/depth/color/points` | `/cmd_vel` |

The profile files are ordinary YAML. Claude Code should change those files if the actual robot uses different topics or frame names.

## Install and build

Prerequisites are Ubuntu 22.04, ROS 2 Humble, the existing Leo simulator or real rover workspace, `colcon`, `rosdep`, and `vcstool`.

```bash
unzip leo_nav2_exploration_bundle.zip
cd leo_nav2_exploration_bundle

# Use /ros2_ws in the supplied Docker workflow, or another Humble workspace.
./scripts/install_dependencies.sh /ros2_ws
./scripts/build_overlay.sh /ros2_ws
```

`install_dependencies.sh` copies only `src/leo_nav2_exploration` into the workspace, imports the pinned `frontier_exploration_ros2` v1.6.1 source dependency, and invokes `rosdep`. It does not edit an existing package.

## First simulation test

Start with LiDAR only. This isolates TF, odometry, SLAM, footprint, planner, and controller behavior before adding depth-camera complexity.

```bash
./scripts/run_sim_doorway.sh /ros2_ws --lidar-only
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

In RViz, display `/map`, the global and local costmaps, `/plan`, the laser scans, and the robot footprint. Send one goal into the opposite room. Do not begin autonomous exploration until one manual goal crosses the doorway and one return goal crosses back.

After the manual test succeeds, run the repeatable eight-crossing regression:

```bash
./scripts/run_sim_doorway.sh /ros2_ws --automated --lidar-only
```

The regression writes `/tmp/leo_nav2_doorway_regression.json`. It requires all eight action goals to succeed. Visual confirmation is still required for zero contacts, stable mapping, and a single final velocity publisher.

Enable the simulator point cloud only after the LiDAR-only regression passes:

```bash
./scripts/run_sim_doorway.sh /ros2_ws --automated --voxel
```

## Existing simulator, without the fixture

When `leo_rover_gazebo/two_robots.launch.py` is already running:

```bash
./scripts/run_sim_navigation.sh /ros2_ws --lidar-only
./scripts/run_preflight.sh /ros2_ws sim_leo1
```

The generic simulator overlay attaches to `leo1`. It does not spawn or control `leo2`.

## Frontier exploration

The frontier process is deliberately cold-idle. First start it:

```bash
./scripts/run_frontier_explorer.sh /ros2_ws sim_leo1
```

Then explicitly enable it:

```bash
./scripts/start_exploration.sh /ros2_ws
```

Stop it at any time:

```bash
./scripts/stop_exploration.sh /ros2_ws
```

To wait for the explorer's completion event and save the map:

```bash
./scripts/explore_and_save.sh /ros2_ws /ros2_ws/maps/easy_room 900
```

The frontier configuration rejects small frontier fragments, candidates too close to the current pose, and repeatedly failing regions. Goal preemption lets the explorer finish a frontier once the LiDAR has revealed it instead of driving to the wall-edge coordinate itself. It is still necessary to inspect the selected frontiers in RViz for the first run.

## Real rover launch

Do not start the overlay on top of the old explorer, another Nav2 instance, another Collision Monitor, or another SLAM Toolbox instance. Stop those processes first. Keep the robot lifted or physically guarded for the first command-chain test.

```bash
./scripts/run_real_navigation.sh /ros2_ws \
  --i-have-stopped-old-navigation \
  --lidar-only
```

Run the read-only audit from another terminal:

```bash
./scripts/run_preflight.sh /ros2_ws real_root
```

Only Collision Monitor should publish final `/cmd_vel`. The command chain is:

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_guarded -> /cmd_vel
```

The real profile limits linear speed to 0.10 m/s and angular speed to 0.30 rad/s. Those limits are starting values, not a substitute for an operator and physical emergency stop.

The depth cloud is opt-in on real hardware:

```bash
./scripts/run_real_navigation.sh /ros2_ws \
  --i-have-stopped-old-navigation \
  --voxel
```

Use `--voxel` only after the point-cloud topic is live and the complete base-to-camera TF chain passes the calibration checks.

## Important tuning files

- `src/leo_nav2_exploration/config/sim/nav2.yaml`
- `src/leo_nav2_exploration/config/real/nav2.yaml`
- `src/leo_nav2_exploration/config/*/scan_filter.yaml`
- `src/leo_nav2_exploration/config/*/slam.yaml`
- `src/leo_nav2_exploration/config/*/collision_monitor.yaml`
- `src/leo_nav2_exploration/config/*/velocity_guard.yaml`
- `src/leo_nav2_exploration/config/*/frontier.yaml`

The initial physical footprint is 0.42 m by 0.42 m with 0.01 m Nav2 padding. Replace it with measured front, rear, left, and right extents before real motion. Keep the same footprint in both global and local costmaps.

## Diagnostics

Record the raw and filtered scan, TF, map, costmaps, plan, and every command-chain stage:

```bash
./scripts/record_debug_bag.sh /ros2_ws sim_leo1 /ros2_ws/bags/door_test
./scripts/record_debug_bag.sh /ros2_ws real_root /ros2_ws/bags/real_test --with-cloud
```

Save the occupancy map and, when available, the SLAM pose graph:

```bash
./scripts/save_map.sh /ros2_ws /ros2_ws/maps/room_01
```

Calibration procedures are in `docs/CALIBRATION.md`. Simulator acceptance and failure isolation are in `docs/SIMULATION_TEST_PLAN.md`.

## Validation boundary

`validate_bundle.sh` runs unit tests, Python compilation, YAML/XML/SDF parsing, shell syntax checks, package-contract checks, and integrity checks. This build environment does not contain ROS 2 Humble or Gazebo, so it cannot prove lifecycle activation, plugin loading, physics, sensor QoS, or real-world clearance. Those are explicitly covered by the supplied simulation and field acceptance steps.
