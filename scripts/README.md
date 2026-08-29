# Scripts

| Script | Purpose |
|--------|---------|
| `setup_workspace.sh` | Fetch the three nested third-party repos + world patch (run first on a new machine) |
| `sim_gpu_wsl.sh` | Start `leo_sim` container with GPU Gazebo (WSL / Docker Desktop) |
| `sim_gpu_linux.sh` | Same, on native Linux with `nvidia-container-toolkit` |
| `slam_nav2_wsl.sh` | SLAM + Nav2 + RViz inside running container |
| `teleop_wsl.sh` | Interactive keyboard teleop (needs `leo_rover_control`, see note) |
| `build_ogre_wsl_gpu.sh` | One-time Ogre GPU patch build into `docker/patched/` |
| `auto_explore_run.sh` | Headless single-rover exploration run |
| `auto_multirobot_run.sh` | Headless one/two-rover run of the integrated stack |
| `demo_teleop_record.sh` | Presentation demo: stack + small rosbag, driven by teleop |
| `demo_teleop_wsl.sh` | Keyboard teleop for the demo (self-contained) |
| `map_coverage.py` | Sample known map area every N seconds into `coverage*.log` |
| `multirobot_sensor_probe.py` | Bounded message-level RGB, depth-cloud, and LiDAR health proof |
| `plot_coverage.py` | `coverage*.log` -> coverage-over-time PNG |
| `render_multirobot_media.py` | Run dir -> maps, coverage curve, path overlay |
| `render_timelapse.py` | Recorded map snapshots -> live merge MP4 |
| `render_collab_comparison.py` | Matched coordinated/independent coverage and duplication figure |

Windows entry points at repo root call these via WSL (`run_sim.ps1`, etc.).

> `teleop_wsl.sh` calls `ros2 run leo_rover_control keyboard_control`. That
> package's `keyboard_control.py` is no longer in `src/` — only the
> console-script shim survives under `install/` — so the entry point raises on
> import from a fresh checkout. Use `demo_teleop_wsl.sh` instead; it runs
> `demo_teleop.py`, which depends on nothing but `rclpy`.

## Coverage over time — running more runs

One line, for anyone who just wants more curves:

```bash
scripts/auto_multirobot_run.sh <coordinated|independent|single> \
    <husarion_office|office_world|depot_world> reports/<your-run-name> [cap_min]
```

## On a new machine, before anything else

`git clone` does not bring the whole workspace. `src/frontier_exploration_ros2`,
`src/husarion_gz_worlds` and `src/m-explore-ros2` are *nested git repositories*,
not submodules, so git skips them -- including the one holding the
`husarion_office` world every multi-robot run uses. The world also carries a
one-line local edit (`real_time_factor` 1.0 -> 2.0) that is not in any repo.

```bash
./scripts/setup_workspace.sh          # clone/checkout at the validated commits, patch the world
./scripts/setup_workspace.sh --check  # verify only
# then, inside the container:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

`install/`, `build/` and `log/` are gitignored, so the workspace must be built
on each machine. `--symlink-install` is not optional: the harness relies on
edits to `src/*.py` taking effect without a rebuild.

## Running

The harness picks the GPU launcher for the host automatically:
`sim_gpu_wsl.sh` under Docker Desktop or WSL, `sim_gpu_linux.sh` on native
Linux. Force one with `SIM_LAUNCHER=/path/to/script`. The difference matters:
under WSL no NVIDIA EGL/GLX ICD reaches the container, so Ogre runs through
Mesa's `d3d12` translation layer (`GL_RENDERER = D3D12 (NVIDIA ...)`, OpenGL
4.2), which segfaults in `libd3d12core.so` after 9-13 min of two-camera
rendering. On native Linux Ogre gets the real driver. Check which path a run
took in `gpu_check.txt`; `llvmpipe` there means software GL and a ~6x slowdown.

Two more things to know before reading a run's numbers. First, exploration
budgets are wall-clock, but `husarion_office` runs at ~0.45x real time on a
laptop (188 mesh models; the Gazebo server is the serial bottleneck, not the
GPU), so a 10-minute cap buys under 5 minutes of simulated exploration. Second,
`finished=N/2` in `run.log` counts only rovers that ran out of frontiers;
rovers that gave up appear as `aborted=` and their maps are incomplete by
definition. A run is only a complete exploration when it logs
`all explorers finished`.

`cap_min` limits exploration (default 14). The harness separately enforces a
20-minute end-to-end wall budget, reserves 150 seconds for maps/media, and has
a container-identity-checked hard watchdog. Override `MAX_WALL_MIN` or
`FINALIZE_RESERVE_SEC` only when deliberately changing that operational limit.

It writes `coverage.log` (merged) and `coverage_leo1.log` / `coverage_leo2.log`
(per rover) into the run directory, sampled every 15 s and clipped to the same
world bounds for every condition. Camera-on two-rover runs also fail closed if
either rover's RGB, camera calibration, organized depth cloud, or LiDAR stream
is silent; the measured rates and message metadata are saved in
`sensor_probe.json`. Final maps, plots, camera stills, and `merge_timelapse.mp4`
are rendered automatically before teardown. Turn raw recordings into pictures
again with either:

```bash
python3 scripts/plot_coverage.py <run>/coverage_leo1.log <run>/coverage.png "label"
python3 scripts/render_multirobot_media.py <run> --world office_world   # all figures
python3 scripts/render_collab_comparison.py office_world <coordinated-run> \
    <independent-run> <comparison.png>
```

Robot count is 1 (`single`) or 2 — `alignment` and `shared_map_merger` are
pairwise leo2→leo1, so a third rover needs code, not a flag. Worlds are the
three above; a new world needs a `SPAWN_POSES` entry in
`src/leo_rover_gazebo/launch/spawn_poses.py` and a `BOUNDS` case in the run
script, or coverage is measured on an unclipped footprint and is not
comparable to the existing numbers.

## Demo recording — small bag, driven by hand

```bash
scripts/demo_teleop_record.sh husarion_office 1      # terminal 1: record
scripts/demo_teleop_wsl.sh 1                         # terminal 2: drive
```

Terminal 1 starts the sim, SLAM, Nav2 (for the costmaps), the coverage /
trajectory / map-time-lapse recorders and a rosbag; terminal 2 gives you
W/A/S/D. Ctrl+C in terminal 1 saves the maps, renders the plots and finalises
the bag.

What is recorded, and why the bag stays in the tens of MB rather than GB:

| in the bag | not in the bag |
|---|---|
| `/leoN/demo/image/compressed` — 320 px JPEG @ 4 Hz | `/leoN/camera/image` (raw 640×480 rgb8 @ 15 Hz ≈ 13 MB/s) |
| `/leoN/demo/map` — full grid @ 0.2 Hz | `/leoN/map` @ 1 Hz |
| `/leoN/{local,global}_costmap/costmap` + `costmap_updates` | depth images, point clouds |
| `/leoN/scan`, `/tf`, `/tf_static`, `/clock`, `/cmd_vel`, `/plan` | `/leoN/aruco/debug_image` |

`scripts/demo_bag_feeds.py` is what produces the two `demo/` topics; the bag is
written with message-level zstd on top. Knobs: `VIDEO_W`, `VIDEO_HZ`, `JPEG_Q`,
`MAP_HZ`, `BAG_COMPRESS=none`, `DURATION_MIN`, `GT_ODOM=true`.

Alongside the bag the run directory gets the things a bag is a bad container
for: `coverage_leoN.log` + `coverage.png` (coverage over time), `traj_leoN.csv`
+ `traj_overlay.png` (path over time), `timelapse/*.npz` (map over time,
re-renderable offline) and `leoN_map.pgm/.yaml` (the final map).

Replay for the film cut:

```bash
ros2 bag play reports/<run>/bag --clock
rviz2 -d config/rviz/demo_rover_local_leo1.rviz
```
