#!/usr/bin/env bash
# Start SLAM + Nav2 + safety chain on rover 2. The robot does NOT move from
# this alone. Port of jetson-04's proven start_stack.sh (2026-08-25).
set -eo pipefail
cd "$(dirname "$0")"
source ./env.sh
mkdir -p logs maps runs

# Camera pointcloud for the costmap camera layer (same RealSense tweak as
# rover 4, namespace rob_2).
timeout 20 ros2 param set /rob_2/camera pointcloud__neon_.enable true \
  || echo "WARN: could not enable camera pointcloud (is leo-hardware up?)"
timeout 20 ros2 param set /rob_2/camera decimation_filter.enable true \
  || echo "WARN: could not enable decimation filter"
timeout 20 ros2 param set /rob_2/camera decimation_filter.filter_magnitude 4 \
  || echo "WARN: could not set decimation magnitude"

if [[ -f logs/stack.pid ]] && kill -0 "$(cat logs/stack.pid)" 2>/dev/null; then
  echo "stack already running (pid $(cat logs/stack.pid)); stop_all.sh first"
  exit 1
fi
setsid nohup ros2 launch leo_nav2_exploration real_navigation.launch.py \
  robot_ns:=rob_2 navigation_start_delay:=25.0 \
  > logs/stack.log 2>&1 < /dev/null &
echo $! > logs/stack.pid
echo "stack starting (pgid $(cat logs/stack.pid)); log: ~/leo_nav2_ws/logs/stack.log"
