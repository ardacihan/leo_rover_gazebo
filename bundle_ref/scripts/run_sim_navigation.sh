#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"

WORKSPACE="$(workspace_path "${1:-}")"
VOXEL=false
START_SLAM=true
for arg in "${@:2}"; do
  case "$arg" in
    --voxel) VOXEL=true ;;
    --lidar-only) VOXEL=false ;;
    --no-slam) START_SLAM=false ;;
    *) fail "Unknown argument: $arg" ;;
  esac
done
source_workspace "$WORKSPACE"
require_ros_package leo_nav2_exploration
exec ros2 launch leo_nav2_exploration sim_navigation.launch.py \
  start_slam:="$START_SLAM" enable_voxel:="$VOXEL"
