# Simulation Test Plan

This plan is designed to determine whether the overlay can be transferred to the real rover. Do not jump directly to frontier exploration. Each stage isolates one component and has an explicit pass condition.

## Stage 0: Static bundle validation

From the extracted bundle:

```bash
./validate_bundle.sh
```

Pass condition: all unit tests, Python compilation, YAML/XML/SDF parsing, shell checks, package-contract checks, and manifest checks pass.

This stage does not load ROS plugins or run physics.

## Stage 1: Build and interface discovery

```bash
./scripts/install_dependencies.sh /ros2_ws
./scripts/build_overlay.sh /ros2_ws
```

Then verify expected interfaces:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 pkg prefix leo_nav2_exploration
ros2 pkg prefix frontier_exploration_ros2
```

Pass condition: both packages are discoverable and the build reports no missing plugin libraries.

## Stage 2: Doorway fixture, LiDAR only

```bash
./scripts/run_sim_doorway.sh /ros2_ws --lidar-only
```

The launch uses the existing empty Gazebo world, places `leo1` at the left side, places `leo2` far away, and inserts a two-room fixture with a 0.78 m clear doorway. The starting padded footprint is 0.44 m wide.

In RViz display:

- `/map`;
- `/leo1/scan` and `/leo1/scan_filtered`;
- `/global_costmap/costmap`;
- `/local_costmap/costmap`;
- `/local_costmap/published_footprint`;
- `/plan`;
- TF and robot model.

Pass conditions before moving:

1. Raw scan is live.
2. Filtered scan is live and does not remove walls outside the rover envelope.
3. The footprint surrounds the full robot.
4. Map, robot, scan, and fixture align.
5. The map remains stable for at least 30 seconds while stationary.
6. `ros2 topic info -v /leo1/cmd_vel` shows only Collision Monitor as publisher after the overlay is active.

Run the automated preflight:

```bash
./scripts/run_preflight.sh /ros2_ws sim_leo1
```

## Stage 3: One manual doorway crossing

Send one goal approximately 1.5 m beyond the doorway using RViz's Nav2 Goal tool. Check the global path before the rover moves.

The path should:

- approach near the doorway centre;
- align the square footprint before the narrowest section;
- cross without rotating against a jamb;
- continue far enough into the second room that the rear clears the frame.

Pass condition: successful action result, zero contact, no oscillatory recovery loop, and no map jump.

Send a return goal to the initial room. Both directions must pass before automated testing.

If the path goes around the wall endpoint, the fixture was not spawned correctly or the map is incomplete. If the planner says no path although the gap is visible, inspect footprint size, map resolution, lethal cells, and scan alignment. Do not reduce the footprint below the robot's measured size.

## Stage 4: Eight-crossing regression

Stop the first launch and run:

```bash
./scripts/run_sim_doorway.sh /ros2_ws --automated --lidar-only
```

The regression waits for startup, captures the current map-frame robot pose, and sends eight alternating goals across the door. It includes small lateral offsets. Results are written to:

```text
/tmp/leo_nav2_doorway_regression.json
```

Acceptance:

- 8 of 8 goals succeed;
- zero Gazebo contacts with the fixture;
- no duplicated walls or room copies;
- at most occasional recovery, not a repeated recovery loop;
- no manual teleoperation;
- final command remains owned by Collision Monitor;
- command velocities remain within profile limits.

The JSON validates action outcomes only. Review Gazebo contacts and the map visually or from logged topics.

## Stage 5: Depth-camera VoxelLayer

Repeat:

```bash
./scripts/run_sim_doorway.sh /ros2_ws --automated --voxel
```

Pass conditions:

- all LiDAR-only acceptance criteria still pass;
- point-cloud obstacles align with the fixture;
- the floor does not become a persistent obstacle;
- stale or absent point clouds do not stop LiDAR navigation;
- camera clearing does not erase LiDAR walls.

If this stage fails but LiDAR-only passes, keep the camera disabled and fix camera TF, cloud frame, min/max obstacle height, and raytrace ranges. Do not retune SLAM.

## Stage 6: Office-room manual navigation

Start the existing office simulation, then attach the overlay:

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py world:=husarion_office
# in another terminal
./scripts/run_sim_navigation.sh /ros2_ws --lidar-only
```

Send deliberate goals to room centres and through visible doors. Test at least:

1. straight open-space goal;
2. doorway goal;
3. goal requiring a 90-degree approach;
4. return through the same doorway;
5. goal near—but not on—a wall;
6. interrupted goal followed by a new goal.

Pass condition: the rover follows map-aware paths without the old random wall-seeking behavior.

## Stage 7: Frontier exploration, cold-idle first

Start the frontier node separately:

```bash
./scripts/run_frontier_explorer.sh /ros2_ws sim_leo1
```

Do not start exploration yet. Inspect:

```text
/explore/frontiers
/explore/selected_frontier
/explore/optimized_map
```

Pass condition: candidates represent meaningful unknown regions, not isolated wall pixels or locations inside inflated obstacles.

Then start:

```bash
./scripts/start_exploration.sh /ros2_ws
```

Acceptance:

- frontier goals are reached through Nav2 rather than direct velocity control;
- failed regions are suppressed after repeated attempts;
- small wall-edge fragments are ignored;
- revealed frontiers can complete before the rover drives directly to the boundary;
- exploration emits `/exploration_complete` when no meaningful reachable frontier remains;
- the rover stops after completion.

Save output:

```bash
./scripts/explore_and_save.sh /ros2_ws /ros2_ws/maps/sim_room 900
```

## Stage 8: Repeatability

Run the entire doorway regression at least five times from clean simulator restarts. A single successful run is insufficient.

Recommended acceptance target before real deployment:

```text
40/40 doorway action goals successful
0 contacts
0 duplicate-room maps
0 competing final cmd_vel publishers
0 stale-data guard bypasses
```

## Debug recording

Record every failed run before changing parameters:

```bash
./scripts/record_debug_bag.sh /ros2_ws sim_leo1 /ros2_ws/bags/failed_door --with-cloud
```

Also capture:

```bash
ros2 param dump /planner_server
ros2 param dump /controller_server
ros2 param dump /local_costmap/local_costmap
ros2 param dump /global_costmap/global_costmap
ros2 param dump /collision_monitor
ros2 node list | sort
ros2 topic info -v /leo1/cmd_vel
```

## Failure isolation

### Map duplicates or rotates while stationary

Likely TF, timestamp, odometry ownership, or SLAM input issue. Stop planner tuning. Verify one `map -> odom`, one `odom -> base`, correct LiDAR transform, and synchronized timestamps.

### Global plan refuses the doorway

Check the measured polygon, padded width, doorway occupied cells, global costmap resolution, and LiDAR alignment. Confirm the State Lattice file loaded. Do not switch to a point robot.

### Global plan is valid but controller oscillates at the doorway

Inspect local costmap alignment and DWB trajectories. Reduce speed before altering footprint. Check rotation-shim engagement thresholds, local inflation, and whether a point-cloud obstacle is appearing in the gap.

### Collision Monitor stops a centred passage

Compare raw/filtered scan and visualized stop/slowdown polygons. A real wall point should remain outside the self-filter. Confirm the robot is centred and that the measured opening really matches the model. Increase doorway width for diagnosis rather than shrinking the safety footprint.

### Rover drives close to walls in open space

Inspect the inflation potential field and global path. A smooth inflation gradient should prefer the centre. Verify the planner is using `cost_penalty` and that the global costmap has an inflation layer. Avoid adding random turn logic.

### Frontier exploration approaches tiny wall gaps

Increase `min_frontier_size_cells`, `frontier_selection_min_distance`, and candidate minimum distance; inspect map leaks and scan alignment. Keep navigation unchanged until the candidate-generation problem is isolated.
