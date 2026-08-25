#!/usr/bin/env bash
# Save the current SLAM map to ~/leo_nav2_ws/maps/<name>.{pgm,yaml}
set -eo pipefail
cd "$(dirname "$0")"
source ./env.sh
name="${1:-rover2_$(date +%Y%m%d_%H%M%S)}"
mkdir -p maps
timeout 30 ros2 run nav2_map_server map_saver_cli -f "maps/$name" \
  --ros-args -p save_map_timeout:=10.0
ls -la maps/ | tail -4
