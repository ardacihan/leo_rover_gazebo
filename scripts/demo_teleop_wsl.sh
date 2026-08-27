#!/usr/bin/env bash
# Interactive keyboard teleop for the demo recording (must run in a real
# terminal -- it puts the tty in raw mode).
#
#   scripts/demo_teleop_wsl.sh [num_robots]
#
# Uses scripts/demo_teleop.py rather than `ros2 run leo_rover_control
# keyboard_control`: that package's keyboard_control.py is not in the tree any
# more, only the console-script shim in install/, so the entry point raises on
# import. teleop_wsl.sh still calls it; this one does not depend on it.
set -eo pipefail

NUM_ROBOTS="${1:-1}"

if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "leo_sim is not running. Start the recording first:" >&2
  echo "  scripts/demo_teleop_record.sh <world> [num_robots]" >&2
  exit 1
fi

echo "Click this terminal, then drive. W/A/S/D, SPACE=stop, 1/2=pick rover, Q=quit."
docker exec -it leo_sim bash -lc "
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  exec python3 /ros2_ws/scripts/demo_teleop.py $NUM_ROBOTS
"
