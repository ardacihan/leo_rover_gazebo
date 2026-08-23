#!/usr/bin/env bash
# Sequential experiment matrix for the collaborative-exploration study.
#
# For each world runs three conditions:
#   single       - 1 rover  (auto_explore_run.sh custom, proven baseline)
#   independent  - 2 rovers, uncoordinated allocation
#   coordinated  - 2 rovers, distributed greedy coordinated allocation
#
# Runs are strictly sequential (they share the leo_sim container name). A run
# whose coverage.log already has samples is skipped, so the batch is
# resumable. Individual failures do not abort the batch.
#
# Usage: run_all_experiments.sh [cap_min] [world1 world2 ...]
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAP="${1:-35}"; shift || true
WORLDS=("$@")
[[ ${#WORLDS[@]} -eq 0 ]] && WORLDS=(office_world depot_world husarion_office)

BASE=reports/collab
LOG() { echo "[run_all $(date +%H:%M:%S)] $*"; }

done_already() {  # $1 = outdir (rel)
  [[ -s "$ROOT/$1/coverage.log" ]] && \
    grep -q 'known=' "$ROOT/$1/coverage.log" 2>/dev/null
}

run_one() {  # $1=condition $2=world $3=outdir
  local cond="$1" world="$2" out="$3"
  if done_already "$out"; then LOG "SKIP $out (already has data)"; return 0; fi
  LOG "START $cond / $world -> $out"
  set +e
  if [[ "$cond" == "single" ]]; then
    bash "$ROOT/scripts/auto_explore_run.sh" custom "$world" "$out" "" "$CAP"
  else
    bash "$ROOT/scripts/auto_collab_run.sh" "$cond" "$world" "$out" "$CAP"
  fi
  local rc=$?
  set -e 2>/dev/null || true
  LOG "END   $cond / $world (rc=$rc)"
  # Safety: pull artifacts out of the (now-stopped) container is not possible,
  # but the bind mount syncs on stop; give it a moment.
  sleep 5
}

for world in "${WORLDS[@]}"; do
  run_one coordinated "$world" "$BASE/${world}_coordinated"
  run_one independent "$world" "$BASE/${world}_independent"
  run_one single      "$world" "$BASE/${world}_single"
done
LOG "ALL DONE"
