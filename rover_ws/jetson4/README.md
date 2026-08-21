# jetson-4 workspace mirror (state as of 2026-08-21 evening)

Replay kit for the `~/leo_nav2_ws` workspace on jetson-4
(`jetson-04@192.168.178.104`, password = username, ROS_DOMAIN_ID=4, SSH via
plink with pinned hostkey — see REAL_ROVER_GUIDE.md). On 2026-08-21 this
setup mapped 82.5 m², found the doorway and drove 10+ m into the corridor
autonomously (evidence: `reports/room_mapping_2026-08-21/`).

## What lives where

Already ON the rover (built in `~/leo_nav2_ws`, survives reboot):

- `src/leo_nav2_exploration/` — identical to this repo's
  `src/leo_nav2_exploration` at the 2026-08-21 state (cloud_filter rover
  fixes, scan_normalizer, tuned `config/real/{nav2,slam,explore}.yaml`).
  Re-deploy after changes: tar the package, scp, `colcon build
  --packages-select leo_nav2_exploration`.
- `src/m-explore-ros2/` — upstream + the vacuous-success blacklist patch.
  The patched files are mirrored here as `explore.cpp` / `explore.h`
  (drop into `src/m-explore-ros2/explore/{src,include/explore}/`, then
  `colcon build --packages-select explore_lite`).
- Session scripts in `~/leo_nav2_ws`: `start_stack.sh` (mirrored here),
  `start_explore.sh` (THE go command), `stop_all.sh`, `preflight.sh`,
  `save_map.sh`, `tf_freshener.py` (re-stamps map->odom; Humble
  slam_toolbox freezes the stamp when stationary), `run_recorder.py`
  (mirrored here), `explore_params.yaml` (copy of
  `config/real/explore.yaml`), `fastdds_udp_only.xml` (UDP-only DDS —
  wedged SHM locks killed endpoints on 2026-08-20).

Mirrored in this directory (workstation-side masters):

- `start_stack.sh` — hardened bringup: stops crash-looping `leo-nav`,
  kills `color_detector` / `exploration_supervisor` / `stuck_recovery`
  (each returns on reboot, each costs up to a core), sets the RealSense
  params that do NOT persist (`pointcloud__neon_.enable`,
  `decimation_filter` magnitude 4), then launches
  `real_navigation.launch.py`.
- `run_recorder.py` — per-2s npz frames: map, global/local costmap, plan,
  pose, frontier markers, NavigateToPose status transitions
  (events.jsonl), 1 Hz 320x180 JPEG video. Run with the output dir as
  argv[1].
- `monitor_run.sh` — workstation-side polling loop (battery, explorer
  liveness, frame progress) with battery abort.
- `explore.cpp` / `explore.h` — the patched m-explore files.

## Run procedure (proven 2026-08-21)

```
bash preflight.sh                      # battery, scan 10 Hz, wheel_odom 20 Hz, odom TF
bash start_stack.sh                    # SLAM+Nav2; wait ~45 s
#   verify: 0 'range readings, expected' in logs/stack.log,
#   /scan_uniform 512 rays, /camera_points_filtered a few Hz,
#   /cmd_vel publisher count == 1, bt_navigator active
setsid nohup python3 tf_freshener.py > logs/freshener.log 2>&1 & echo $! > logs/freshener.pid
setsid nohup python3 run_recorder.py ~/leo_nav2_ws/runs/<name> > logs/recorder.log 2>&1 & echo $! > logs/recorder.pid
bash start_explore.sh                  # ROBOT MOVES
# ... end of run:
kill -INT -- -$(cat logs/explore.pid); bash save_map.sh <name>; bash stop_all.sh
```

Afterwards pull `runs/<name>` and render it:
`python scripts/render_run.py <run_dir> --every 10 --gif`
(3 panels per frame: SLAM map with red occupied cells, global costmap with
orange penalty halos + red lethal cells, local costmap; plus path, plan,
frontiers, goal).

## Known traps

- Kill process GROUPS (`kill -INT -- -pgid`); killing a `ros2 run` wrapper
  PID leaves the node running as an orphan.
- Fresh `ros2 topic hz`/`tf2_echo` probes often claim topics are missing
  when they are fine; cross-check with `ros2 topic echo --once`.
- Preempted Nav2 goals report ABORTED on Humble — a healthy exploring run
  shows almost all goals as aborted. Only worry about instant-SUCCEEDED
  storms (that was the run-1 livelock, now patched).
- Never `--log-level debug` on the Jetson (froze sshd once; power cycle).
- User-approved battery floor: ~10.0 V (preflight's 11 V line is advisory).
- At session end 2026-08-21 sshd stopped answering while ping worked —
  check/power-cycle the rover before the next session.
