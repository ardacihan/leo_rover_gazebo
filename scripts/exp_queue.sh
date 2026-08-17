#!/usr/bin/env bash
# Run a serial queue of experiments, scoring each one as it finishes.
#
# Runs are deliberately serialized: only one Gazebo instance at a time, so
# real-time factor -- and therefore how far each stack gets inside its
# wall-clock cap -- stays comparable across runs.
#
# Usage:
#   exp_queue.sh <cap_min> <spec> [<spec> ...]
#     spec = <stack>:<world>[:<profile>]
#   e.g. exp_queue.sh 35 orig:husarion_office bundle:office_world:realistic
set -uo pipefail

CAP_MIN="${1:?usage: exp_queue.sh <cap_min> <stack:world[:profile]> ...}"
shift
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QLOG="$ROOT/reports/exp/queue.log"
mkdir -p "$ROOT/reports/exp"

LOG() { echo "[queue $(date +%H:%M:%S)] $*" | tee -a "$QLOG"; }

LOG "queue start: $* (cap ${CAP_MIN}min each)"

for spec in "$@"; do
  IFS=':' read -r stack world profile <<< "$spec"
  profile="${profile:-realistic}"
  tag="$(printf '%s_%s_%s' "$stack" "$world" "$profile")"
  out="reports/exp/${tag}"

  if [[ -f "$ROOT/$out/map.yaml" ]]; then
    LOG "SKIP $tag (already has a saved map)"
    continue
  fi

  LOG "START $tag"
  bash "$ROOT/scripts/exp_run.sh" "$stack" "$world" "$out" "$profile" "$CAP_MIN" \
    >> "$ROOT/$out.log" 2>&1
  rc=$?
  # exit 3 just means the explorer never declared completion before the cap,
  # which is a normal outcome for a time-boxed run, not a failure.
  LOG "END   $tag rc=$rc"

  LOG "SCORE $tag"
  bash "$ROOT/scripts/exp_score.sh" "$out" "$world" >> "$ROOT/$out.score.log" 2>&1 \
    || LOG "WARNING: scoring failed for $tag"

  # Make sure nothing is left holding the GPU/CPU before the next run starts.
  docker stop leo_sim >/dev/null 2>&1
  sleep 10
done

LOG "queue complete"
