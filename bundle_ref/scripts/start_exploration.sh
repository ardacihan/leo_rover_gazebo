#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
WORKSPACE="$(workspace_path "${1:-}")"
source_workspace "$WORKSPACE"
require_ros_package frontier_exploration_ros2
ros2 service list | grep -qE '(^|/)control_exploration$' || fail "Cold-idle frontier_explorer is not running. Start run_frontier_explorer.sh first."
exec ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
