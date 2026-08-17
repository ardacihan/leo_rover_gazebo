#!/usr/bin/env bash
# Shared helpers for the operator scripts. Source this file; do not execute it directly.

BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

workspace_path() {
  local requested="${1:-${LEO_ROS_WORKSPACE:-$HOME/ros2_ws}}"
  mkdir -p "$requested"
  (cd "$requested" && pwd)
}

source_humble() {
  [[ -r /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble was not found at /opt/ros/humble/setup.bash"
  # setup.bash may inspect unset variables, so temporarily relax nounset.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  [[ "${ROS_DISTRO:-}" == "humble" ]] || fail "This baseline is pinned to ROS 2 Humble"
}

source_workspace() {
  local workspace="$1"
  source_humble
  [[ -r "$workspace/install/setup.bash" ]] || fail "Workspace is not built: $workspace/install/setup.bash is missing"
  set +u
  source "$workspace/install/setup.bash"
  set -u
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "Required command '$1' is not installed"
}

require_ros_package() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || fail "ROS package '$1' is not visible in the sourced environment"
}
