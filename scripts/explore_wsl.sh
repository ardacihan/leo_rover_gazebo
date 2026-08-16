#!/usr/bin/env bash
# Autonomous frontier exploration (explore_lite) inside leo_sim.
# Requires the sim (./run_sim.ps1) and SLAM+Nav2 (./run_slam_nav2.ps1) running.
set -eo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "leo_sim is not running. Start the sim first: ./run_sim.ps1"
  exit 1
fi

docker exec -d leo_sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  exec ros2 launch leo_rover_gazebo explore.launch.py
'

echo "explore_lite started in leo_sim"
echo "Pause/resume: ros2 topic pub --once /explore/resume std_msgs/msg/Bool \"{data: false}\""
echo "Watch progress in RViz (map grows on its own)."
