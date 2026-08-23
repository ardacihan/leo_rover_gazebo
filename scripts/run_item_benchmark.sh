#!/usr/bin/env bash
# Item-search benchmark matrix: 3 conditions x 2 seeds on office_world.
#
#   1robot  - single rover item search (baseline)
#   2indep  - two rovers, no coordination, no claim sharing
#   2coord  - two rovers, coordinated allocation + shared item/coverage claims
#
# Seeds = two marker layouts (A/B). Runs are sequential (shared container
# name + full-fidelity RTF); a run with an existing non-empty items.jsonl is
# skipped so the batch is resumable.
#
# Usage: run_item_benchmark.sh [cap_min] [base_outdir]
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAP="${1:-20}"
BASE="${2:-reports/item_search_collab}"
WORLD=office_world
MDIR=/ros2_ws/src/leo_rover_exploration/config
declare -A MARKERS=(
  [seedA]="$MDIR/mock_markers_office_world.yaml"
  [seedB]="$MDIR/mock_markers_office_world_b.yaml"
)

run() {  # run <n> <mode> <cond> <seed>
  local n="$1" mode="$2" cond="$3" seed="$4"
  local out="$BASE/${cond}_${seed}"
  if [[ -s "$ROOT/$out/items.jsonl" ]]; then
    echo "[benchmark] SKIP $cond $seed (items.jsonl exists)"
    return 0
  fi
  echo "[benchmark] ===== $cond $seed (n=$n mode=$mode) ====="
  "$ROOT/scripts/auto_item_run.sh" "$n" "$mode" "$WORLD" \
      "${MARKERS[$seed]}" "$out" "$CAP" \
      > "$ROOT/$BASE/${cond}_${seed}.out" 2>&1
  local rc=$?
  echo "[benchmark] $cond $seed done (rc=$rc)"
  sleep 5
}

mkdir -p "$ROOT/$BASE"
for seed in seedA seedB; do
  run 2 coordinated 2coord "$seed"
  run 2 independent 2indep "$seed"
  run 1 independent 1robot "$seed"
done
echo "[benchmark] matrix complete"
