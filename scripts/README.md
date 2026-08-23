# Scripts

| Script | Purpose |
|--------|---------|
| `sim_gpu_wsl.sh` | Start `leo_sim` container with GPU Gazebo |
| `slam_nav2_wsl.sh` | SLAM + Nav2 + RViz inside running container |
| `teleop_wsl.sh` | Interactive keyboard teleop |
| `build_ogre_wsl_gpu.sh` | One-time Ogre GPU patch build into `docker/patched/` |

Windows entry points at repo root call these via WSL (`run_sim.ps1`, etc.).
