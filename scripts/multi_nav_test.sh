#!/usr/bin/env bash
# Functional Nav2 test for the 2-robot stack: send each rover a goal ~1.2 m
# ahead (in the merged map frame) and confirm it plans + moves.
set -o pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

pose() {  # echo "x y" of map->$1/base_link
  timeout 4 ros2 run tf2_ros tf2_echo map "$1/base_link" 2>/dev/null \
    | grep -m1 -A1 'Translation' | grep -oE '\[.*\]' \
    | tr -d '[]' | awk -F, '{print $1, $2}'
}

send() {  # $1=ns $2=gx $3=gy  (background)
  ros2 action send_goal "/$1/navigate_to_pose" nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $2, y: $3, z: 0.0}, orientation: {w: 1.0}}}}" \
    > "/tmp/nav_$1.log" 2>&1 &
}

declare -A X0 Y0
for ns in leo1 leo2; do
  read x y <<< "$(pose $ns)"
  X0[$ns]=$x; Y0[$ns]=$y
  gx=$(awk "BEGIN{print $x + 1.2}")
  echo "$ns start=($x, $y) goal=($gx, $y)"
  send $ns "$gx" "$y"
done

echo "navigating 25s..."
sleep 25

for ns in leo1 leo2; do
  read x y <<< "$(pose $ns)"
  moved=$(awk "BEGIN{dx=$x-${X0[$ns]}; dy=$y-${Y0[$ns]}; print sqrt(dx*dx+dy*dy)}")
  echo "$ns now=($x, $y) moved=${moved} m"
done
echo "=== goal logs ==="
tail -3 /tmp/nav_leo1.log /tmp/nav_leo2.log
