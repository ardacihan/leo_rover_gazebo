#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
[[ $# -ge 2 ]] || fail "Usage: $0 WORKSPACE OUTPUT_PREFIX [TIMEOUT_SECONDS]"
WORKSPACE="$(workspace_path "$1")"
PREFIX="$2"
TIMEOUT_SECONDS="${3:-900}"
[[ "$TIMEOUT_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "Timeout must be numeric"
source_workspace "$WORKSPACE"
require_ros_package frontier_exploration_ros2
ros2 service list | grep -qE '(^|/)control_exploration$' || fail "Cold-idle frontier_explorer is not running"

LOG="$(mktemp)"
cleanup() { rm -f "$LOG"; }
trap cleanup EXIT
set +e
timeout "$TIMEOUT_SECONDS" ros2 topic echo --once /exploration_complete std_msgs/msg/Empty >"$LOG" 2>&1 &
LISTENER_PID=$!
set -e
sleep 0.5
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start

set +e
wait "$LISTENER_PID"
STATUS=$?
set -e
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop || true
"$BUNDLE_ROOT/scripts/save_map.sh" "$WORKSPACE" "$PREFIX"

if [[ $STATUS -eq 124 ]]; then
  printf 'Exploration timed out after %s seconds; a partial map was saved.\n' "$TIMEOUT_SECONDS" >&2
  exit 4
elif [[ $STATUS -ne 0 ]]; then
  cat "$LOG" >&2
  fail "Completion listener failed with status $STATUS; map was still saved"
fi
printf 'Exploration completion event received and map saved.\n'
