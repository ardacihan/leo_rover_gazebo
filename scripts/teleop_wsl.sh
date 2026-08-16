#!/usr/bin/env bash
# Interactive keyboard teleop (must run in a real terminal).
set -eo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "leo_sim is not running. Start the sim first: ./run_sim.ps1"
  exit 1
fi

echo "Click this terminal, then use W/A/S/D to drive. Space=stop, Q=quit."
docker exec -it leo_sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  exec ros2 run leo_rover_control keyboard_control --ros-args -p use_sim_time:=true
'
