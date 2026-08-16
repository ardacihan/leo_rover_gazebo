#!/usr/bin/env bash
set -o pipefail
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select leo_rover_exploration 2>&1 | tail -4
source install/setup.bash
echo "=== py_compile frontier_explorer ==="
python3 -m py_compile src/leo_rover_exploration/leo_rover_exploration/frontier_explorer.py \
  && echo "compile OK"
echo "=== pytest ==="
cd src/leo_rover_exploration && python3 -m pytest test/ -q 2>&1 | tail -10
