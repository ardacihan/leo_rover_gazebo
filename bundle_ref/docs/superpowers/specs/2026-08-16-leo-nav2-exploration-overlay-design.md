# Leo Nav2 Exploration Overlay Design

## Goal

Provide a standalone ROS 2 Humble overlay that can be copied into the existing workspace without modifying the current simulator or real-rover packages. The overlay must replace reactive wandering with map-aware Nav2 navigation, use the rover's polygon footprint, pass doorway regression tests in simulation, support LiDAR-only SLAM with an optional RealSense local voxel layer, and expose repeatable calibration and preflight tooling.

## Profiles

- `sim_leo1`: uses `/leo1/scan`, `/leo1/odom`, `/leo1/camera/points`, `leo1/base_link`, `leo1/odom`, and publishes the final command to `/leo1/cmd_vel`.
- `real_root`: uses root-level topics and frames, leaves odometry and static TF ownership to the existing robot bringup, and publishes the final command to `/cmd_vel` through a guarded Collision Monitor chain.

## Architecture

SLAM Toolbox owns `map -> odom`. Existing robot or simulator odometry owns `odom -> base`. Nav2 uses Smac State Lattice with the full polygon footprint and DWB with `ObstacleFootprint`. The controller output passes through Nav2 Velocity Smoother, a fail-closed velocity guard, and Nav2 Collision Monitor. Collision Monitor is the only final velocity publisher.

LiDAR is the only SLAM input. LiDAR also feeds 2D obstacle layers. PointCloud2 from the depth camera is optional and contributes only to the local VoxelLayer. Camera absence or invalid depth must never veto otherwise valid LiDAR navigation and must never clear LiDAR obstacles by inference.

## Exploration policy

Frontier exploration is present but starts cold-idle. The operator must first prove manual Nav2 doorway traversal. Exploration rejects tiny frontiers, highly inflated or blocked candidates, trivially close targets, and repeatedly failing regions. Completion is based on exhaustion of meaningful reachable frontiers, after which a helper can save the occupancy map and SLAM pose graph.

## Geometry and calibration

The default footprint is a centered 0.42 m square, expressed as independently configurable front, rear, left, and right extents. Calibration tools report recommended YAML values but do not silently modify TF. The required physical checks are footprint extents, LiDAR XY/yaw, camera height/roll/pitch, doorway clear width, and straight/rotation odometry scale.

## Validation boundary

The bundle is statically validated in this environment: unit tests, Python compilation, YAML/XML parsing, shell syntax, package metadata, launch-source compilation, and configuration invariants. ROS 2 lifecycle activation, Gazebo physics, and real hardware motion remain acceptance tests for the user's ROS 2 Humble system.
