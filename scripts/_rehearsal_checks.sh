#!/bin/bash
# Remaining rehearsal checks, run INSIDE the leo_sim container against the
# already-running dual real_mapping stack.
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
OUT=/ros2_ws/reports/night_2026-08-25/phase4_rehearsal
V="$OUT/rehearsal_summary.txt"

for ns in leo1 leo2; do
  b=$(timeout 8 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A3 'position:' | grep 'x:' | head -1 | grep -oE '[-0-9.e]+')
  (timeout 10 ros2 topic pub -r 5 /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{linear: {x: 0.15}}' >/dev/null 2>&1 || true)
  ros2 topic pub --once /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  a=$(timeout 8 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A3 'position:' | grep 'x:' | head -1 | grep -oE '[-0-9.e]+')
  moved=$(python3 -c "import sys
try:
    b,a=float('$b'),float('$a')
    print('yes' if abs(a-b)>0.15 else 'no')
except Exception:
    print('no')")
  if [ "$moved" = yes ]; then
    echo "CHECK safety-chain motion($ns): PASS (odom x $b -> $a)" | tee -a "$V"
  else
    echo "CHECK safety-chain motion($ns): FAIL (odom x '$b' -> '$a')" | tee -a "$V"
  fi
done

sleep 8
for ns in leo1 leo2; do
  if timeout 8 ros2 run tf2_ros tf2_echo $ns/map $ns/base_link 2>/dev/null | grep -q 'Translation'; then
    echo "CHECK tf($ns/map -> $ns/base_link): PASS" | tee -a "$V"
  else
    echo "CHECK tf($ns/map -> $ns/base_link): FAIL" | tee -a "$V"
  fi
done

for ns in leo1 leo2; do
  ros2 run nav2_map_server map_saver_cli -f "$OUT/${ns}_map" --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map >>"$OUT/map_saver.log" 2>&1 \
    || echo "map save ($ns) failed" | tee -a "$V"
done

fails=$(grep -c FAIL "$V" || true)
if [ "${fails:-0}" -eq 0 ]; then echo "REHEARSAL PASSED" | tee -a "$V"; else echo "REHEARSAL FAILED ($fails checks)" | tee -a "$V"; fi
