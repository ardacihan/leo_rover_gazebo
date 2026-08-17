#!/usr/bin/env bash
# Re-score every saved bench run with the current eval_map.py.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_SDF=/ros2_ws/src/leo_rover_gazebo/worlds/office_world.sdf

docker run --rm -v "$ROOT:/ros2_ws" --entrypoint bash leo_rover_humble -lc "
  cd /ros2_ws/scripts
  for d in /ros2_ws/reports/slam_bench/*/ /ros2_ws/reports/slam_bench_verify/*/; do
    name=\$(basename \$d)
    [[ -f \$d/map.yaml ]] || continue
    echo \"--- \$name ---\"
    python3 eval_map.py \$d/map.yaml $WORLD_SDF \
      --pose-csv \$d/pose_error.csv --png \$d/eval.png \
      --json \$d/metrics.json --label \$name || true
  done"
