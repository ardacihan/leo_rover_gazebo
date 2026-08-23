#!/usr/bin/env bash
# Multi-seed comparison of the candidates that are still in contention.
#
# Candidate C turned out to be unreliable across seeds (0.25-0.80), so nothing
# gets recommended on a single run again. Every candidate here is scored on
# three independent odometry-noise draws, all on the self-filtered scan and at
# the nominal 12% skid-steer yaw error.
#
#   D_strict     - C's loop-closure reach, with strict acceptance restored
#   E_reach_only - B plus ONLY the loop-closure reach change
#
# B and C already have seeds 1-2 and 1-3 respectively from earlier sweeps.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTROOT="${1:-reports/slam_bench_seeds}"
CAND=/ros2_ws/scripts/slam_candidates
SCAN=/leo1/scan_real_selffiltered
WORLD_SDF=/ros2_ws/src/leo_rover_gazebo/worlds/office_world.sdf

LOG() { echo "[seeds $(date +%H:%M:%S)] $*"; }

for name in D_strict E_reach_only; do
  case "$name" in
    D_strict)     cfg="$CAND/D_strict.yaml" ;;
    E_reach_only) cfg="$CAND/E_reach_only.yaml" ;;
  esac
  for seed in 1 2 3; do
    run="${name}_s${seed}"
    if [[ -f "$ROOT/$OUTROOT/$run/map.yaml" ]]; then
      LOG "skip $run"; continue
    fi
    LOG "=== $run ==="
    SCAN_TOPIC="$SCAN" YAW_SCALE=0.12 SLIP=0.01 SEED="$seed" \
      "$ROOT/scripts/slam_bench_run.sh" "$run" realistic "$cfg" \
      "$OUTROOT/$run" office_world office_full 25 || LOG "$run FAILED"
  done
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
