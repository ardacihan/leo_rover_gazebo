# Leo Rover Gazebo (ROS 2 Humble)

Leo Rover simulation in Gazebo Harmonic with SLAM, Nav2, and keyboard teleop.  
Optimized for **Windows + WSL2 + Docker Desktop** with **GPU rendering** via WSLg.

## Repository layout

```
leo_rover_gazebo/
├── run_sim.ps1              # Start Gazebo + Leo (GPU)
├── run_slam_nav2.ps1        # SLAM + Nav2 + RViz
├── run_teleop.ps1           # Keyboard drive (interactive terminal)
├── run_explore.ps1          # Autonomous exploration (explore_lite)
├── run_explore_custom.ps1   # Autonomous exploration (custom frontier explorer)
├── Dockerfile
├── docker-entrypoint.sh
├── docker/
│   ├── ogre-wsl-gpu.patch   # Ogre fix for WSL D3D12 OpenGL
│   └── patched/             # Built .so files (gitignored)
├── maps/                    # Saved maps land here
├── scripts/
│   ├── sim_gpu_wsl.sh
│   ├── slam_nav2_wsl.sh
│   ├── teleop_wsl.sh
│   ├── explore_wsl.sh
│   ├── explore_custom_wsl.sh
│   └── build_ogre_wsl_gpu.sh
└── src/                     # ROS 2 packages
    ├── leo_rover_gazebo/
    ├── leo_rover_description/
    ├── leo_rover_control/
    ├── leo_rover_exploration/   # custom frontier explorer (multi-robot ready)
    ├── m-explore-ros2/          # explore_lite + map_merge (cloned)
    ├── leo_common-ros2/
    └── leo_rover_bringup/   # upstream tutorial examples
```

## Prerequisites

- Windows 11 with WSL2 (Ubuntu) and Docker Desktop (WSL integration enabled)
- NVIDIA GPU drivers on Windows (for hardware OpenGL in WSLg)
- Docker image built once: `docker build -t leo_rover_humble .`

### Office world

```bash
git clone https://github.com/husarion/husarion_gz_worlds.git src/husarion_gz_worlds
```

### Build workspace (inside container or WSL)

```bash
docker run --rm -v "$(pwd):/ros2_ws" -w /ros2_ws leo_rover_humble \
  bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

### GPU rendering (one-time, ~10 min)

WSLg uses D3D12-backed OpenGL; stock Ogre crashes without a patch:

```bash
./scripts/build_ogre_wsl_gpu.sh
```

This writes `docker/patched/RenderSystem_GL3Plus.so*` (gitignored). The sim mounts them at runtime.

## Quick start (PowerShell)

```powershell
.\run_sim.ps1            # Gazebo GUI + Leo in husarion office
.\run_slam_nav2.ps1      # SLAM + Nav2 costmaps + RViz
.\run_teleop.ps1         # WASD drive (separate interactive window)
.\run_explore.ps1        # OR: autonomous mapping with explore_lite
.\run_explore_custom.ps1 # OR: autonomous mapping with the custom explorer
```

Stop everything: `docker stop leo_sim`

### Choosing a world

The sim launch accepts a world name (searched in `husarion_gz_worlds/worlds`
and `leo_rover_gazebo/worlds`) or a full path:

```bash
WORLD=husarion_world ./scripts/sim_gpu_wsl.sh   # default: husarion_office
GUI=false WORLD=leo_world ./scripts/sim_gpu_wsl.sh  # headless
```

### Navigation

1. Drive around with teleop to build the map (or use an already-mapped office).
2. In RViz, use the **Nav2 Goal** tool to send autonomous goals.

### Autonomous exploration

Two implementations, both drive Nav2 goals while slam_toolbox maps:

- **explore_lite** (`run_explore.ps1`): greedy frontier exploration from
  [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2).
  Params: `src/leo_rover_gazebo/config/explore_params_leo.yaml`.
- **Custom frontier explorer** (`run_explore_custom.ps1`):
  `leo_rover_exploration` package — frontier clustering + scoring,
  progress watchdog with goal blacklisting, return-to-start, and automatic
  map saving (`maps/explored_map.*`) when done. Publishes frontier claims on
  `/exploration_claims` so a second rover can avoid duplicating work
  (groundwork for distributed multi-robot exploration).
  Params: `src/leo_rover_exploration/config/frontier_explorer_leo1.yaml`.

Both verified end-to-end (2026-06-11): explore_lite mapped `husarion_office`
(~113 m², `maps/office_explore_lite.*`), the custom explorer mapped
`leo_world` — now a multi-room 20×20 m arena with doorways and obstacles —
to full coverage (`maps/world_custom_explorer.*`). Each map ships as
`.pgm`/`.yaml` (Nav2 static map) plus `.posegraph`/`.data` (slam_toolbox
serialized graph for continuing SLAM or localization mode).
Note: `husarion_world.sdf` is just a giant Husarion logo mesh — not useful
for exploration tests.

## Architecture notes

- **Split launch** (`two_robots_gpu.launch.py`): Gazebo server (software GL, stable bridges) + GUI client (GPU via patched Ogre + `/usr/lib/wsl` libs).
- **Odometry**: ground-truth `OdometryPublisher` plugin (wheel odom drifts when slipping on walls).
- **Teleop**: `leo_rover_control` publishes `/leo1/cmd_vel` at 10 Hz.

## Linux / WSL shell

From repo root in Ubuntu WSL:

```bash
./scripts/sim_gpu_wsl.sh
./scripts/slam_nav2_wsl.sh
./scripts/teleop_wsl.sh
```
