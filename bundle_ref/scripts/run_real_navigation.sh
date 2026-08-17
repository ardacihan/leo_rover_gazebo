#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"

WORKSPACE="$(workspace_path "${1:-}")"
ACK=false
FORCE=false
VOXEL=false
START_SLAM=true
for arg in "${@:2}"; do
  case "$arg" in
    --i-have-stopped-old-navigation) ACK=true ;;
    --force) FORCE=true ;;
    --voxel) VOXEL=true ;;
    --lidar-only) VOXEL=false ;;
    --no-slam) START_SLAM=false ;;
    *) fail "Unknown argument: $arg" ;;
  esac
done
[[ "$ACK" == true ]] || fail "Refusing real motion. Stop the old explorer/Nav2/SLAM stack, then pass --i-have-stopped-old-navigation"
source_workspace "$WORKSPACE"
require_ros_package leo_nav2_exploration

if [[ "$FORCE" != true ]]; then
  EXISTING="$(ros2 node list 2>/dev/null || true)"
  for pattern in safe_room_explorer collision_monitor controller_server planner_server bt_navigator velocity_smoother; do
    if grep -Eq "(^|/)$pattern$" <<<"$EXISTING"; then
      fail "Existing node matching '$pattern' is running. Stop the old navigation stack or explicitly add --force after checking ownership."
    fi
  done
  if [[ "$START_SLAM" == true ]] && grep -Eq '(^|/)slam_toolbox$' <<<"$EXISTING"; then
    fail "An existing slam_toolbox node is running. Stop it, or use --no-slam only when that one is intentionally retained."
  fi
fi

exec ros2 launch leo_nav2_exploration real_navigation.launch.py \
  start_slam:="$START_SLAM" enable_voxel:="$VOXEL"
