#!/usr/bin/env bash
# Phase-A bring-up verification for the 2-robot collaborative stack.
# Jogs both rovers to seed SLAM, then reports maps + TF chain health.
set -o pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "=== jogging both rovers (seed SLAM) ==="
for ns in leo1 leo2; do
  ( timeout 6 ros2 topic pub -r 5 /$ns/cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.14}}" >/dev/null 2>&1 ) &
done
wait
for ns in leo1 leo2; do
  ros2 topic pub --once /$ns/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
done
sleep 6

echo "=== map topics (rate) ==="
for t in /leo1/map /leo2/map /map; do
  n=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo "NO DATA")
  echo "  $t : $n"
done

echo "=== merged /map info ==="
ros2 topic echo /map --field info --once 2>/dev/null | head -20 || echo "  no merged map yet"

echo "=== TF chain map -> leo{i}/base_link ==="
for ns in leo1 leo2; do
  out=$(timeout 5 ros2 run tf2_ros tf2_echo map $ns/base_link 2>/dev/null \
        | grep -A1 -m1 'Translation' || echo "  BROKEN")
  echo "  map -> $ns/base_link:"
  echo "$out" | sed 's/^/    /'
done
echo "=== done ==="
