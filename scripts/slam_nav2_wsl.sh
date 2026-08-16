#!/usr/bin/env bash
# SLAM + Nav2 (costmaps, goals) + RViz inside leo_sim.
set -eo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "leo_sim is not running. Start the sim first: ./run_sim.ps1"
  exit 1
fi

docker exec -d leo_sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  export DISPLAY="${DISPLAY:-:0}"
  export QT_X11_NO_MITSHM=1
  export XDG_RUNTIME_DIR=/tmp/runtime-dir
  mkdir -p /tmp/runtime-dir
  exec ros2 launch leo_rover_gazebo slam_nav2_rviz.launch.py
'

echo "SLAM + Nav2 + RViz started in leo_sim"
echo ""
echo "Drive to build the map: ./run_teleop.ps1"
echo "Set goals in RViz with the Nav2 Goal tool (flag icon)."
