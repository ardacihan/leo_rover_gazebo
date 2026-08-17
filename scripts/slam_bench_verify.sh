#!/usr/bin/env bash
# Follow-up runs for the winning configuration (candidate C on the
# self-filtered scan). Two questions:
#
#   C_lownoise - does the lidar noise level change the answer? Ivan reports the
#                real unit is not especially noisy, so re-run at sigma 5 mm
#                instead of the pessimistic 20 mm default.
#   C_harsh    - how much odometry margin is there? Raise the skid-steer yaw
#                scale error from 12% to 20% and change the noise seed.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTROOT="${1:-reports/slam_bench_verify}"
CFG=/ros2_ws/scripts/slam_candidates/C_robust.yaml
SCAN=/leo1/scan_real_selffiltered
WORLD_SDF=/ros2_ws/src/leo_rover_gazebo/worlds/office_world.sdf

LOG() { echo "[verify $(date +%H:%M:%S)] $*"; }

if [[ ! -f "$ROOT/$OUTROOT/C_lownoise/map.yaml" ]]; then
  LOG "=== C_lownoise (sigma 5 mm) ==="
  SCAN_TOPIC="$SCAN" RANGE_NOISE=0.005 DROPOUT=0.01 \
    "$ROOT/scripts/slam_bench_run.sh" C_lownoise realistic "$CFG" \
    "$OUTROOT/C_lownoise" office_world office_full 25 || LOG "C_lownoise FAILED"
fi

if [[ ! -f "$ROOT/$OUTROOT/C_harsh/map.yaml" ]]; then
  LOG "=== C_harsh (yaw scale 20%, new seed) ==="
  SCAN_TOPIC="$SCAN" YAW_SCALE=0.20 SLIP=0.02 SEED=7 \
    "$ROOT/scripts/slam_bench_run.sh" C_harsh realistic "$CFG" \
    "$OUTROOT/C_harsh" office_world office_full 25 || LOG "C_harsh FAILED"
fi

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
