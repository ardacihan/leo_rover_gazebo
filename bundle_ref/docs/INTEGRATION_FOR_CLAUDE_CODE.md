# Integration Instructions for Claude Code

Use this document as the integration contract. The overlay should remain a separate package. Do not copy individual files into `leo_rover_real_bringup` or replace the existing simulator configuration.

## 1. Copy and build

From the extracted bundle:

```bash
./scripts/install_dependencies.sh /ros2_ws
./scripts/build_overlay.sh /ros2_ws
```

Equivalent manual operations are:

```bash
cp -a src/leo_nav2_exploration /ros2_ws/src/
cd /ros2_ws
vcs import src < /path/to/bundle/dependencies.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/leo_nav2_exploration src/frontier_exploration_ros2 \
  --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to frontier_exploration_ros2 leo_nav2_exploration
```

Do not modify the pinned frontier source until the baseline has been tested.

## 2. Preserve ownership boundaries

The overlay intentionally does not publish any robot transforms. Existing bringup must own:

```text
robot_state_publisher or one static-TF source: base -> lidar/camera links
one odometry source:                         odom -> base
SLAM Toolbox started by this overlay:        map -> odom
```

There must be one publisher for `/map`, one owner for `odom -> base`, and one final velocity publisher. Do not start:

- `safe_room_explorer.py`;
- the old `safe_mapping.launch.py`;
- an old Nav2 graph;
- a second `collision_monitor`;
- a second `slam_toolbox` when `start_slam:=true`;
- `wheel_odom_tf.py` when the robot bridge already publishes the odometry transform;
- another static LiDAR or camera transform.

The overlay's Collision Monitor must be the sole final command publisher.

## 3. Confirm simulator interfaces

The stock `sim_leo1` profile expects:

```text
/leo1/scan                         sensor_msgs/msg/LaserScan
/leo1/scan_filtered                produced by this overlay
/leo1/odom                         nav_msgs/msg/Odometry
/leo1/camera/points                sensor_msgs/msg/PointCloud2
leo1/odom -> leo1/base_link        live TF
/leo1/cmd_vel                      Gazebo bridge input
```

Run:

```bash
ros2 topic type /leo1/scan
ros2 topic type /leo1/odom
ros2 topic type /leo1/camera/points
ros2 run tf2_ros tf2_echo leo1/odom leo1/base_link
```

If any name differs, change all corresponding entries in:

```text
config/sim/nav2.yaml
config/sim/slam.yaml
config/sim/scan_filter.yaml
config/sim/collision_monitor.yaml
config/sim/velocity_guard.yaml
leo_nav2_exploration/preflight_check.py
```

The filtered scan must remain a separate topic. The filter input is the raw sensor topic; SLAM, costmaps, guard, and Collision Monitor all consume the filtered topic.

## 4. Confirm real-rover interfaces

The stock `real_root` profile expects:

```text
/scan                                      raw LaserScan
/scan_filtered                             overlay output
/wheel_odom                                Odometry
base_footprint                             robot base frame
odom                                       odometry frame
/camera/camera/depth/color/points          optional PointCloud2
/cmd_vel                                   bridge/firmware command input
```

The repository evidence shows multiple possible battery namespaces and transform conventions. Do not infer them. Obtain live values with:

```bash
ros2 topic list -t | sort
ros2 node list | sort
ros2 run tf2_tools view_frames
ros2 topic info -v /scan
ros2 topic info -v /wheel_odom
ros2 topic info -v /cmd_vel
```

Change the real profile consistently if the robot uses `base_link`, a namespaced odometry topic, a different LiDAR frame, or a different point-cloud topic.

## 5. Set the measured footprint

Run the calculator after measuring the four extents from the base-frame origin:

```bash
ros2 run leo_nav2_exploration footprint_tool \
  --front 0.21 --rear 0.21 --left 0.21 --right 0.21 \
  --padding 0.01 --door-width 0.78
```

Copy the returned footprint string to both `local_costmap` and `global_costmap` in both Nav2 profiles. Keep `footprint_padding` explicit. Update the scan-filter box only if the measured robot envelope differs materially from the starting 0.42 m square. The self-filter box should remove robot-mounted returns, not enlarge the robot's navigation footprint.

Do not replace the polygon with `robot_radius`. Doorway planning depends on orientation-aware collision checking of the square rover.

## 6. Keep the command chain intact

Simulator:

```text
controller_server and behavior_server
  -> /leo1/cmd_vel_nav
velocity_smoother
  -> /leo1/cmd_vel_smoothed
velocity_guard
  -> /leo1/cmd_vel_guarded
collision_monitor
  -> /leo1/cmd_vel
```

Real rover uses the same suffixes at root level. Never remap the controller, behavior server, or velocity smoother directly to final `cmd_vel`. Recovery motions must pass through the same guard and Collision Monitor.

## 7. Baseline test order

Use this order and do not combine stages:

1. Build and run `validate_bundle.sh`.
2. Launch the doorway fixture with `--lidar-only` and no automatic regression.
3. Verify stationary TF and map stability.
4. Send one manual goal through the doorway.
5. Send one manual return goal.
6. Run all eight automated crossings LiDAR-only.
7. Repeat with `--voxel`.
8. Run the generic office simulator.
9. Start frontier exploration cold-idle and inspect candidates.
10. Enable frontier exploration.
11. Only after all simulator checks pass, adapt `real_root` topics and geometry.
12. On hardware, validate zero-command ownership before sending a goal.

If the LiDAR-only test fails, do not tune the camera. If the map duplicates while stationary, stop and fix TF/odometry ownership before tuning Nav2.

## 8. Parameters intended for first tuning

The baseline deliberately exposes a small set of meaningful parameters:

- Footprint points and padding.
- `inflation_radius` and `cost_scaling_factor`.
- DWB `max_vel_x`, `max_vel_theta`, acceleration limits, and critic scales.
- State Lattice `cost_penalty`, `rotation_penalty`, and reverse policy.
- Collision Monitor zone dimensions and approach time.
- VoxelLayer height and range limits.
- Frontier minimum size, minimum distance, and suppression thresholds.

Do not initially tune loop-closure thresholds, dozens of DWB critics, or camera semantics simultaneously. Change one parameter group, replay the same doorway scenario, and compare bags.

## 9. Expected integration edits

Claude Code should usually edit only:

```text
config/real/nav2.yaml
config/real/slam.yaml
config/real/scan_filter.yaml
config/real/collision_monitor.yaml
config/real/velocity_guard.yaml
config/real/frontier.yaml
leo_nav2_exploration/preflight_check.py
```

Launch code should remain unchanged unless the robot uses a fundamentally different command transport. The convenience launch accepts `start_slam`, `enable_voxel`, `autostart`, `use_respawn`, and `navigation_start_delay` arguments.

## 10. Runtime evidence to collect

For every failed doorway attempt, collect:

```bash
./scripts/record_debug_bag.sh /ros2_ws real_root /ros2_ws/bags/door_failure --with-cloud
ros2 param dump /controller_server
ros2 param dump /planner_server
ros2 param dump /local_costmap/local_costmap
ros2 param dump /global_costmap/global_costmap
ros2 param dump /collision_monitor
ros2 node list | sort
ros2 topic info -v /cmd_vel
```

Do not report a fix as successful based only on a map screenshot. Require repeated physical or Gazebo crossings, zero contacts, no unexpected final-command publishers, and a stable map.
