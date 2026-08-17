#!/usr/bin/env bash
# Close out a run that is already in flight inside leo_sim: wait out the
# remaining time (or an early completion), save the map, flush the recorders so
# their video/CSV artefacts are written, and stop the container.
#
# Usage:
#   exp_finish.sh <outdir-rel-to-repo> <wait_min> [done_pattern]
set -eo pipefail

OUT="$1"; WAIT_MIN="${2:-30}"; DONE_PATTERN="${3:-Exploration (complete|finished|stopped)}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[[ -n "$OUT" ]] || { echo "usage: $0 <outdir> <wait_min> [done_pattern]" >&2; exit 2; }

LOG() { echo "[finish $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }
in_sim() { docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

LOG "waiting up to ${WAIT_MIN} min for completion"
finished=""
deadline=$(( $(date +%s) + WAIT_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died mid-run"; exit 1
  fi
  if grep -qE "$DONE_PATTERN" "$ROOT/$OUT/explorer.log" 2>/dev/null; then
    finished=1; LOG "explorer reports completion"; break
  fi
done
[[ -n "$finished" ]] || LOG "cap reached; collecting artefacts anyway"

in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 10

LOG "saving map"
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/map \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed"

LOG "flushing recorders"
in_sim 'pkill -INT -f "pose_error_recorder[.]py" || true; pkill -INT -f "map_recorder[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true' || true
sleep 25
docker stop leo_sim >/dev/null
LOG "done -> $OUT"
