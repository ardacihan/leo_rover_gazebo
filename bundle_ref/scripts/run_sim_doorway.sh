#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"

WORKSPACE="$(workspace_path "${1:-}")"
AUTOMATED=false
VOXEL=false
for arg in "${@:2}"; do
  case "$arg" in
    --automated) AUTOMATED=true ;;
    --voxel) VOXEL=true ;;
    --lidar-only) VOXEL=false ;;
    *) fail "Unknown argument: $arg" ;;
  esac
done
source_workspace "$WORKSPACE"
require_ros_package leo_nav2_exploration
require_ros_package leo_rover_gazebo
require_ros_package ros_gz_sim

exec ros2 launch leo_nav2_exploration sim_doorway_regression.launch.py \
  run_regression:="$AUTOMATED" enable_voxel:="$VOXEL"
