#!/usr/bin/env bash
# Stop exploration and the nav stack; hold zero velocity. Sensors
# (leo-hardware, lidar wrapper) are left running.
cd "$(dirname "$0")"
source ./env.sh
pkill -f 'explore_lit[e]' && echo "killed explorer" || true
pkill -f 'aruco_detecto[r]' && echo "killed aruco" || true
for p in stack; do
  if [[ -f logs/$p.pid ]]; then
    kill -- -"$(cat logs/$p.pid)" 2>/dev/null || kill "$(cat logs/$p.pid)" 2>/dev/null || true
  fi
done
pkill -f 'real_navigation.launch.p[y]' || true
sleep 2
pkill -f 'nav2|slam_toolbox' 2>/dev/null || true
timeout 8 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
echo "stopped; zero velocity held"
