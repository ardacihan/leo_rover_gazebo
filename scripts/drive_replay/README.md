# drive_replay — real-rover bag review pipeline

Turns a rosbag2 drive recording from the physical Leo Rover into a scrubbable
dashboard with two synchronized views:

* **Default view** — what the sensors recorded: camera, depth, lidar scans
  accumulated on raw odometry (drift left visible on purpose), the exact
  driven path, speed and battery.
* **Rover-stack view** — the same drive replayed through the ivan-branch
  `leo_nav2_exploration` real-rover bundle (scan filter → slam_toolbox →
  Nav2 costmaps/NavFn planner/RPP controller → explore_lite), in shadow mode:
  the bag drives the robot, the stack perceives, maps, picks frontier goals
  and plans in real time. Its `/cmd_vel` output is recorded as
  `/cmd_vel_shadow` so every place the safety chain (velocity guard +
  collision monitor) would have slowed or stopped the human driver is visible
  against what the driver actually commanded.

## One command

    wsl -d Ubuntu -- bash scripts/drive_replay/process_drive_wsl.sh drive_2026-08-20

Steps it runs (each usable alone):

| step | script | output |
|---|---|---|
| extract default view | `extract_bag.py <bag> <out>` | color/depth/lidar.mp4, lidar_map.png, data.json |
| shadow replay (real-time) | `replay_drive_wsl.sh <bag> <out>` | `shadow_bag/`, stack.log |
| render replay | `extract_replay.py <shadow_bag> <metadata.yaml> <out>` | map/global_costmap/local_costmap.mp4, map_final.png, logic.json |
| dashboard | `build_drive_dashboard.py <run_dir>` | dashboard.html |

All videos share one tick grid (8 fps from the bag's own start time), so the
dashboard scrubs every panel with a single slider.

## Baseline vs tuned A/B

`config/real_baseline_2026-08-20/` is a frozen snapshot of the parameters as
deployed for the 2026-08-20 drives (selectable on the rover as
`profile:=real_baseline`). To replay a bag under it:

    bash replay_drive_wsl.sh <bag> <out> real_baseline_2026-08-20

The default third argument `real` replays the current tuning, which since
2026-08-20 adds: a separate `camera_obstacle_layer` (lidar clearing no
longer erases camera marks for low obstacles), the `cloud_filter` node
(ground removal + voxel + outlier rejection -> `/camera_points_filtered`,
enabling `min_obstacle_height` 0.035), and slam `occupancy_threshold` 0.25
(kills the speckle dots the costmap was inflating into no-go disks).

## Replay-mode details

* `replay_stack.launch.py` launches the exact real-rover node set with the
  `config/real` YAMLs, patched only for replay: `use_sim_time: true`
  everywhere (the bag is played with `--clock`), the velocity guard reads the
  bag's driven `/cmd_vel`, and the collision monitor publishes to
  `/cmd_vel_shadow`.
* `depth_to_points.py` rebuilds the RealSense point cloud
  (`/camera/camera/depth/color/points`) from the bag's compressed depth +
  CameraInfo so the costmap camera ObstacleLayer runs exactly as on hardware.
* Nav2 goals come from `explore_lite`; since the robot's motion is the bag's,
  Nav2 keeps replanning ("Failed to make progress" in the events log is the
  expected shadow-mode signature, not a bug).
* Lifecycle managers stay on the wall clock with bonds disabled — the bag
  clock does not tick until play starts.

## Bag assumptions (see probe_bag.py)

`/scan` (laser_frame, mounted yaw π), `/merged_odom` (= odom→base_footprint
TF), `/bag/color/compressed` (jpeg), `/bag/depth/compressed` (16UC1 PNG mm),
`/rob_4/camera/depth/camera_info`, `/tf`, `/tf_static`, `/cmd_vel`.
