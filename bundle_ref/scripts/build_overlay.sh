#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"

WORKSPACE="$(workspace_path "${1:-}")"
source_humble
require_command colcon
[[ -f "$WORKSPACE/src/leo_nav2_exploration/package.xml" ]] || fail "Run install_dependencies.sh first"
[[ -f "$WORKSPACE/src/frontier_exploration_ros2/package.xml" ]] || fail "Pinned frontier dependency is missing"

cd "$WORKSPACE"
colcon build --symlink-install --packages-up-to frontier_exploration_ros2 leo_nav2_exploration

set +u
source "$WORKSPACE/install/setup.bash"
set -u
ros2 pkg prefix leo_nav2_exploration >/dev/null
ros2 pkg prefix frontier_exploration_ros2 >/dev/null
printf '\nBuild completed and both packages are discoverable.\n'
