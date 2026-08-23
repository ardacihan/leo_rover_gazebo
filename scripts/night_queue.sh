#!/usr/bin/env bash
# Run the night's experiment queue back to back on the WSL GPU simulator.
#
# One simulator at a time, deliberately: two would share the ROS domain and the
# CPU, and every timing-sensitive result would be worthless. Each entry is
# `name|env-assignments|stack|world|cap_min|pre-command`. The pre-command runs
# on the Windows side before the run and is how a config variant is selected
# (the planner swap is a YAML edit, not an environment variable); the sync step
# inside exp_run_wsl.sh then carries it into the WSL workspace. A failing run
# does not stop the
# queue -- exp_run_wsl.sh exits 3 when the explorer never declared completion,
# which is a result, not an error.
#
# Usage: scripts/night_queue.sh <queue-file> [outdir-root]

set -uo pipefail
QUEUE="${1:?usage: night_queue.sh <queue-file> [outdir-root]}"
ROOT_OUT="${2:-reports/night}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGFILE="$ROOT/$ROOT_OUT/queue.log"
mkdir -p "$ROOT/$ROOT_OUT"

LOG() { echo "[queue $(date +%H:%M:%S)] $*" | tee -a "$LOGFILE"; }

# One queue at a time, enforced rather than assumed. Two queues sharing the ROS
# domain put two simulators on one gz-transport partition: `/clock` stops
# advancing, every node stays alive and quiet, and the runs look like
# exploration stalls in the metrics. Six runs were lost to exactly that, and
# the cause was mistaken for the WSL distro degrading over time because
# `pkill` on the stale queue had silently failed to kill it.
LOCK="$ROOT/$ROOT_OUT/.queue.lock"
if ! mkdir "$LOCK" 2>/dev/null; then
  echo "another night_queue.sh holds $LOCK -- refusing to start a second" >&2
  echo "if that is stale: rmdir '$LOCK'" >&2
  exit 1
fi
trap 'rmdir "$LOCK" 2>/dev/null' EXIT

# Read the queue on fd 3, not stdin. `wsl.exe` inside the loop inherits stdin
# and consumes the rest of the file, so a stdin-fed loop silently runs exactly
# one entry and then reports "queue complete".
while IFS='|' read -r name envs stack world cap pre <&3; do
  [[ -z "${name// }" || "$name" == \#* ]] && continue
  out="$ROOT_OUT/$name"
  if [[ -f "$ROOT/$out/map_score.json" ]]; then
    LOG "skip $name (already scored)"
    continue
  fi
  if [[ -n "${pre// }" ]]; then
    LOG "pre: $pre"
    ( cd "$ROOT" && eval "$pre" ) >>"$LOGFILE" 2>&1 || LOG "WARNING: pre-command failed"
  fi
  LOG "start $name : $envs $stack $world cap=${cap}m"
  start=$(date +%s)
  ( eval "export $envs"; bash "$ROOT/scripts/exp_run_wsl.sh" "$stack" "$world" "$out" realistic "$cap" ) \
    >"$ROOT/$out.log" 2>&1
  rc=$?
  LOG "finish $name rc=$rc in $(( ($(date +%s) - start) / 60 )) min"

  LOG "scoring $name"
  bash "$ROOT/scripts/exp_score.sh" "$out" "$world" >"$ROOT/$out.score.log" 2>&1 \
    || LOG "scoring failed for $name"
  if [[ -f "$ROOT/$out/aruco_registry.json" && "$world" == "office_world" ]]; then
    python "$ROOT/scripts/score_aruco.py" "$ROOT/$out/aruco_registry.json" \
      "$ROOT/src/leo_rover_exploration/config/mock_markers_office_world.yaml" \
      --json-out "$ROOT/$out/aruco_score.json" >"$ROOT/$out.aruco.log" 2>&1 || true
  fi
done 3< "$QUEUE"

LOG "queue complete"
python "$ROOT/scripts/night_table.py" "$ROOT/$ROOT_OUT" | tee -a "$LOGFILE"
