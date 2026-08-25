#!/usr/bin/env bash
# THE go command: starts frontier exploration. The robot WILL start moving.
set -eo pipefail
cd "$(dirname "$0")"
source ./env.sh
mkdir -p logs
if [[ -f logs/explore.pid ]] && kill -0 "$(cat logs/explore.pid)" 2>/dev/null; then
  echo "explorer already running (pid $(cat logs/explore.pid))"
  exit 1
fi
setsid nohup ros2 run explore_lite explore --ros-args \
  --params-file "$HOME/leo_nav2_ws/explore_params.yaml" \
  > logs/explore.log 2>&1 < /dev/null &
echo $! > logs/explore.pid
echo "explorer started (pgid $(cat logs/explore.pid)); ROBOT WILL MOVE."
