#!/usr/bin/env bash
# Repeatability and drift-margin study for the winning configuration.
#
# Everything so far is single runs, and the odometry noise draw is
# timing-dependent, so run-to-run spread is unknown. C_harsh also failed at a
# 20% yaw-scale error, but it changed three things at once (yaw scale, slip and
# seed), so that needs disentangling.
#
#   C_seed2 / C_seed3  - winner at nominal drift, new seeds -> repeatability
#   B_seed2            - baseline at a new seed -> is the gap bigger than noise
#   C_yaw16 / C_yaw20  - yaw scale only, slip and seed held at nominal
#                        -> where does the configuration actually break
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTROOT="${1:-reports/slam_bench_margin}"
C=/ros2_ws/scripts/slam_candidates/C_robust.yaml
B=/ros2_ws/scripts/slam_candidates/B_real_branch.yaml
SCAN=/leo1/scan_real_selffiltered
WORLD_SDF=/ros2_ws/src/leo_rover_gazebo/worlds/office_world.sdf

LOG() { echo "[margin $(date +%H:%M:%S)] $*"; }

# name | cfg | yaw_scale | seed
RUNS=(
  "C_seed2|$C|0.12|2"
  "C_seed3|$C|0.12|3"
  "B_seed2|$B|0.12|2"
  "C_yaw16|$C|0.16|1"
  "C_yaw20|$C|0.20|1"
)

for row in "${RUNS[@]}"; do
  IFS='|' read -r name cfg yaw seed <<< "$row"
  if [[ -f "$ROOT/$OUTROOT/$name/map.yaml" ]]; then
    LOG "skip $name"; continue
  fi
  LOG "=== $name (yaw_scale=$yaw seed=$seed) ==="
  SCAN_TOPIC="$SCAN" YAW_SCALE="$yaw" SEED="$seed" SLIP=0.01 \
    "$ROOT/scripts/slam_bench_run.sh" "$name" realistic "$cfg" \
    "$OUTROOT/$name" office_world office_full 25 || LOG "$name FAILED"
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
