#!/usr/bin/env bash
# Poll the rover every ~40 s during exploration run 2.
# Exit 0 explore done/returned, 2 low battery (<10.0 V), 3 explorer died, 1 timeout.
PLINK="/c/Program Files/PuTTY/plink.exe"
HK="SHA256:S94JtCyPgJifWCIZO/Z/t5gi1RAxiQZnH1LeZFgyzMU"
for i in $(seq 1 13); do
  out=$("$PLINK" -batch -ssh -hostkey "$HK" -pw jetson-04 jetson-04@192.168.178.104 '
    source /opt/ros/humble/setup.bash 2>/dev/null
    export ROS_DOMAIN_ID=4
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/leo_nav2_ws/fastdds_udp_only.xml
    b=$(timeout 8 ros2 topic echo /rob_2/firmware/battery_averaged --once 2>/dev/null | grep -m1 "data:" | awk "{print \$2}")
    echo "BATT=$b"
    if kill -0 $(cat ~/leo_nav2_ws/logs/explore.pid 2>/dev/null) 2>/dev/null; then echo EXPLORE=alive; else echo EXPLORE=dead; fi
    tail -3 ~/leo_nav2_ws/logs/explore.log
    ls ~/leo_nav2_ws/runs/run_2026-08-21_2/frame_*.npz 2>/dev/null | tail -1
    grep -c "blacklisting" ~/leo_nav2_ws/logs/explore.log 2>/dev/null | sed "s/^/BLACKLISTS=/"
    uptime | grep -o "load average.*"
  ' 2>/dev/null)
  echo "=== poll $i $(date +%H:%M:%S)"
  echo "$out"
  batt=$(echo "$out" | sed -n 's/^BATT=\([0-9.]*\).*/\1/p' | head -1)
  if echo "$out" | grep -qiE "no more frontier|All frontiers traversed|returning to initial|returned to initial"; then
    echo "MONITOR_RESULT=EXPLORE_DONE"; exit 0
  fi
  if echo "$out" | grep -q "EXPLORE=dead"; then
    echo "MONITOR_RESULT=EXPLORER_DEAD"; exit 3
  fi
  if [ -n "$batt" ] && awk "BEGIN{exit !($batt < 10.0)}"; then
    echo "MONITOR_RESULT=LOW_BATTERY"; exit 2
  fi
  sleep 40
done
echo "MONITOR_RESULT=TIMEOUT"
exit 1
