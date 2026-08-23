#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
WORKSPACE="$(workspace_path "${1:-}")"
PROFILE="${2:-sim_leo1}"
source_workspace "$WORKSPACE"
require_ros_package frontier_exploration_ros2
exec ros2 launch leo_nav2_exploration frontier_exploration.launch.py \
  profile:="$PROFILE" autostart:=false
