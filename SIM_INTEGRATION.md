# Two-rover integration: what is wired, what is not

Branch `feat/multi-robot-integration` = `fix/real-rover-mapping` (field-validated,
2026-08-21) + `ivan-branch` (sim collab) + `main` (shared mapping, already
contained in the fix branch). Goal: prove the two-rover pipeline in Gazebo so the
next lab session is assembly, not development.

## The one contract that makes this work

All the tag consumers speak **`visualization_msgs/MarkerArray` on
`/{robot}/tag_detections`**, with the tag id in `Marker.id`:

| Node | Package | Publishes | Status |
|---|---|---|---|
| `apriltag_detection_node` | `multi_robot_shared_mapping` | `/{ns}/tag_detections` | sim reference, AprilTag |
| `aruco_detector` | `leo_nav2_exploration` | param `detection_topic` | **hardware-validated, use this** |
| `aruco_detection_node` | `leo_rover_semantic_vision` | `String` status + TF only | not usable for alignment |

So swapping the sim AprilTag node for the real ArUco detector is **configuration,
not code**: launch `aruco_detector` with `detection_topic:=/leo1/tag_detections`.
`tag_based_map_aligner` cannot tell the difference.

`aruco_detector` is already sim-aware — `frame_is_optical` exists precisely for
this (`true` for a RealSense, **`false` for Gazebo**, whose `rgbd_camera` stamps
images with the link frame rather than an optical frame). Getting it wrong
silently rotates every detection by 90 degrees instead of erroring.

## Pipeline for the sim proof of concept

```
two_robots.launch.py          spawns leo1 + leo2 + ArUco models (per-robot overrides)
  -> slam_multi.launch.py     one slam_toolbox per rover -> /leo{i}/map
  -> aruco_detector x2        -> /leo{i}/tag_detections
  -> tag_based_map_aligner    -> /estimated_transform/leo2_to_leo1
  -> map_based_aligner        -> /map_based_transform/leo2_to_leo1  (cross-check)
  -> shared_map_merger        -> /shared_map  (frame leo1/map)
```

All three stacks already agree on the `leo1` / `leo2` namespaces. That was luck,
and it is the main reason this merge is cheap.

## Choices made, and the redundancy left behind

Two map mergers survived the merge. **Use `multi_robot_shared_mapping`** — it is
parameterized, has a confidence signal, fuses a tag estimate with a map-matching
estimate, and carries `compare_to_ground_truth` / `ground_truth_{x,y,yaw}`
parameters, which is exactly how you score a taped-floor run. `map_compositor.py`
+ `map_fusion.py` (ivan) assume **identity** map offsets, which is only true in
Gazebo because `OdometryPublisher` reports world-frame pose; that assumption does
not survive contact with real hardware. Keep it for its log-odds fusion and
correlative drift registration, which are worth porting later.

They do not collide by default: the shared merger publishes `/shared_map`, the
compositor publishes `/map`. Do not run both.

Similarly, coordinated exploration exists twice: `exploration_policy.py`
(`multi_robot_shared_mapping`, confidence-tiered) and `coordination.py`
(`leo_rover_exploration`, greedy allocation + proximity discount). Both are pure
functions with no ROS imports and both are unit-tested. Neither is wired to
`explore_lite`, which is what the real rover actually runs.

## Known mismatches to fix before the lab

1. **Marker size and dictionary disagree.** `aruco_detector` defaults to
   `marker_length: 0.15`, `dictionary: DICT_4X4_50`, `allowed_ids: [1..8]`.
   `tools/make_aruco_print_pdf.py` prints **80 mm** markers from
   **`DICT_4X4_1000`, ids 0-9**. Pick one and set all three parameters. Per the
   detector's own comment, `marker_length` is "the one number a deployment can
   get wrong without anything erroring" — the pose just lands short or long along
   the view ray. Set `samples_file` and check against known positions.
2. **`navigation_overlay.launch.py` hardcodes `/rob_4/camera/depth/color/points`**
   as the cloud filter input. Fine for one rover; must become a launch argument
   before two rovers run it.
3. **`explore_lite` has no coordination hook.** For the 8-hour session, partition
   space physically instead (different start areas / a wall) and treat that as
   spatial partitioning, not coordinated allocation.
4. **`min_tags: 2`** in `tag_based_map_aligner` — either place enough markers in
   the shared area or lower it and lean on the map-matching cross-check.

## Real-rover files that must not be edited casually

Byte-identical to `fix/real-rover-mapping` and carrying the only hardware
evidence there is: `config/real/*`, `config/real_baseline_2026-08-20/*`,
`navigation_overlay.launch.py`, `real_navigation.launch.py`, `cloud_filter.py`
(NEON fast path, raw depth-1 subscription, `spin_thread` TF listener),
`scan_normalizer.py`, `rover_ws/jetson4/`.
