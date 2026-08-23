#!/usr/bin/env bash
# Custom frontier explorer (leo_rover_exploration) inside leo_sim.
# Requires the sim (./run_sim.ps1) and SLAM+Nav2 (./run_slam_nav2.ps1) running.
set -eo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "leo_sim is not running. Start the sim first: ./run_sim.ps1"
  exit 1
fi

docker exec -d leo_sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  exec ros2 launch leo_rover_exploration frontier_explorer.launch.py
'

echo "Custom frontier explorer started in leo_sim"
echo "Status:   ros2 topic echo /frontier_explorer/status"
echo "The map is saved automatically when exploration completes."
