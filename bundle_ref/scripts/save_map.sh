#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
[[ $# -ge 2 ]] || fail "Usage: $0 WORKSPACE OUTPUT_PREFIX"
WORKSPACE="$(workspace_path "$1")"
PREFIX="$2"
source_workspace "$WORKSPACE"
mkdir -p "$(dirname "$PREFIX")"
ros2 run nav2_map_server map_saver_cli -f "$PREFIX"

if ros2 service list -t 2>/dev/null | grep -q '^/slam_toolbox/serialize_map \[slam_toolbox/srv/SerializePoseGraph\]'; then
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${PREFIX}_posegraph'}" || printf 'WARNING: pose-graph serialization failed; occupancy map was saved.\n' >&2
fi
printf 'Saved map prefix: %s\n' "$PREFIX"
