#!/usr/bin/env bash
# Run the SLAM benchmark matrix and score every result against the world.
#
# Each row is: <name> <profile> <candidate-yaml> [SCAN_TOPIC override]
# Runs are strictly sequential -- slam_bench_run.sh owns the leo_sim container.
#
# Usage: slam_bench_sweep.sh [outdir-root] [world] [route] [cap_min]
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTROOT="${1:-reports/slam_bench}"
WORLD="${2:-office_world}"
ROUTE="${3:-office_full}"
CAP="${4:-75}"
CAND=/ros2_ws/scripts/slam_candidates
WORLD_SDF="/ros2_ws/src/leo_rover_gazebo/worlds/${WORLD}.sdf"

LOG() { echo "[sweep $(date +%H:%M:%S)] $*"; }

# name | profile | candidate | scan topic override
RUNS=(
  "A_ideal|ideal|$CAND/A_sim_baseline.yaml|"
  "A_real|realistic|$CAND/A_sim_baseline.yaml|"
  "B_real|realistic|$CAND/B_real_branch.yaml|"
  "B_real_selffiltered|realistic|$CAND/B_real_branch.yaml|/leo1/scan_real_selffiltered"
  "C_real|realistic|$CAND/C_robust.yaml|"
  "C_real_selffiltered|realistic|$CAND/C_robust.yaml|/leo1/scan_real_selffiltered"
  "C_miscal|miscal|$CAND/C_robust.yaml|/leo1/scan_real_selffiltered"
)

for row in "${RUNS[@]}"; do
  IFS='|' read -r name profile cfg scan <<< "$row"
  out="$OUTROOT/$name"
  if [[ -f "$ROOT/$out/map.yaml" ]]; then
    LOG "skip $name (already has a map)"
    continue
  fi
  LOG "=== $name ==="
  if ! SCAN_TOPIC="$scan" "$ROOT/scripts/slam_bench_run.sh" \
        "$name" "$profile" "$cfg" "$out" "$WORLD" "$ROUTE" "$CAP"; then
    LOG "run $name FAILED; continuing"
  fi
done

LOG "scoring"
docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true
docker run --rm -v "$ROOT:/ros2_ws" --entrypoint bash leo_rover_humble -lc "
  cd /ros2_ws/scripts
  for d in /ros2_ws/$OUTROOT/*/; do
    name=\$(basename \$d)
    [[ -f \$d/map.yaml ]] || continue
    echo \"--- \$name ---\"
    python3 eval_map.py \$d/map.yaml $WORLD_SDF \
      --pose-csv \$d/pose_error.csv --png \$d/eval.png \
      --json \$d/metrics.json --label \$name || true
  done" 2>&1 | tee "$ROOT/$OUTROOT/scores.txt"

LOG "done -> $OUTROOT/scores.txt"
