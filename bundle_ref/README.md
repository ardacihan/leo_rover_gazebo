# Leo Nav2 Exploration Bundle

Start with [`docs/README.md`](docs/README.md). The bundle contains a standalone ROS 2 Humble overlay package, pinned frontier dependency metadata, simulation doorway regression, calibration tools, operator scripts, and static validation.

Quick start:

```bash
./validate_bundle.sh
./scripts/install_dependencies.sh /ros2_ws
./scripts/build_overlay.sh /ros2_ws
./scripts/run_sim_doorway.sh /ros2_ws --lidar-only
```
