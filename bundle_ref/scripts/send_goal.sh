#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
[[ $# -ge 4 ]] || fail "Usage: $0 WORKSPACE X_M Y_M YAW_DEG [extra navigate_goal arguments]"
WORKSPACE="$(workspace_path "$1")"
X="$2"; Y="$3"; YAW="$4"; shift 4
source_workspace "$WORKSPACE"
exec ros2 run leo_nav2_exploration navigate_goal "$X" "$Y" "$YAW" "$@"
