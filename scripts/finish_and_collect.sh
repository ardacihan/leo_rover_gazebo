#!/usr/bin/env bash
# Poll the (detached) exploration in leo_sim until both explorers finish or
# coverage plateaus, then save the merged map and sync artifacts to the host.
D="${1:-reports/collab_clean/office_coordinated}"
HOST=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
prev=0; stable=0
for i in $(seq 1 40); do
  n=$(docker exec leo_sim grep -c 'Exploration finished' /ros2_ws/$D/explorer.log 2>/dev/null | tr -d '\r'); n=${n:-0}
  cov=$(docker exec leo_sim grep -oE 'known=[0-9.]+' /ros2_ws/$D/coverage.log 2>/dev/null | tail -1 | grep -oE '[0-9.]+'); cov=${cov:-0}
  echo "[$i] finished=$n/2 coverage=${cov}m2"
  done_num=$(awk "BEGIN{print ($cov==$prev)?1:0}")
  [ "$done_num" = "1" ] && stable=$((stable+1)) || stable=0
  prev=$cov
  if [ "$n" -ge 2 ] || [ "$stable" -ge 4 ]; then echo "DONE"; break; fi
  sleep 30
done
docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$D/merged_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true" 2>&1 | tail -1
docker exec leo_sim pkill -INT -f 'map_recorder[.]py' 2>/dev/null || true
sleep 4
docker cp leo_sim:/ros2_ws/$D/. "$HOST/$D/" 2>/dev/null && echo "collected -> $D"
