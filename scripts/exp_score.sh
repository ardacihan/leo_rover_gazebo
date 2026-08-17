#!/usr/bin/env bash
# Score one exp_run.sh output directory: map quality + driving safety + a
# side-by-side PNG for visual verification. Runs inside the analysis container
# (leo_build) because scipy/opencv live there, not on the Windows host.
#
# Usage:
#   exp_score.sh <outdir-rel-to-repo> <world>
set -eo pipefail

OUT="$1"; WORLD="$2"
CONTAINER="${ANALYSIS_CONTAINER:-leo_build}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$OUT" || -z "$WORLD" ]]; then
  echo "usage: $0 <outdir> <world>" >&2
  exit 2
fi

# Accept a bare world name or a path, mirroring two_robots_gpu.launch.py.
case "$WORLD" in
  *.sdf) WORLD_PATH="$WORLD" ;;
  *)
    if [[ -f "$ROOT/src/husarion_gz_worlds/worlds/$WORLD.sdf" ]]; then
      WORLD_PATH="/ros2_ws/src/husarion_gz_worlds/worlds/$WORLD.sdf"
    else
      WORLD_PATH="/ros2_ws/src/leo_rover_gazebo/worlds/$WORLD.sdf"
    fi
    ;;
esac

docker exec "$CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  cd /ros2_ws/scripts
  echo '===== MAP QUALITY ====='
  if [[ -f /ros2_ws/$OUT/map.yaml ]]; then
    python3 eval_map.py /ros2_ws/$OUT/map.yaml $WORLD_PATH \
      --png /ros2_ws/$OUT/map_vs_world.png \
      --json /ros2_ws/$OUT/map_score.json \
      --pose-csv /ros2_ws/$OUT/pose_error.csv 2>&1 | tail -30
  else
    echo 'NO MAP SAVED'
  fi
  echo
  echo '===== DRIVING SAFETY ====='
  if [[ -f /ros2_ws/$OUT/pose_error.csv ]]; then
    python3 analyze_run_safety.py /ros2_ws/$OUT/pose_error.csv $WORLD_PATH \
      --json /ros2_ws/$OUT/safety_score.json 2>&1 | tail -25
  else
    echo 'NO POSE CSV'
  fi
"

echo
echo "artefacts in $OUT:"
ls -la "$ROOT/$OUT" | awk '{print "  " $5 "  " $9}' | tail -20
