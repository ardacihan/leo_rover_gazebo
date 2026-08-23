#!/usr/bin/env bash
# Bring up 2-robot sim + SLAM + map_merge, jog both rovers, then report
# whether each rover's live pose sits on free vs occupied merged-map cells.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

echo "[diag] starting sim (2 rovers)"
WORLD="${WORLD:-office_world}" GUI=false NUM_ROBOTS=2 "$ROOT/scripts/sim_gpu_wsl.sh"
for i in $(seq 1 40); do
  in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo2/scan$"' && break || sleep 5
done

echo "[diag] slam + merge"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=2 > /ros2_ws/reports/diag_slam.log 2>&1"
in_sim_bg "exec ros2 launch leo_rover_gazebo map_merge_leo.launch.py > /ros2_ws/reports/diag_merge.log 2>&1"
sleep 10

echo "[diag] jog both rovers (drive ~2 m into the environment)"
in_sim 'for ns in leo1 leo2; do (timeout 14 ros2 topic pub -r 5 /$ns/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}" >/dev/null 2>&1 &); done; sleep 16; for ns in leo1 leo2; do ros2 topic pub --once /$ns/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true; done'
sleep 5

echo "[diag] alignment check:"
in_sim 'python3 /ros2_ws/scripts/check_alignment.py leo1,leo2'

echo "[diag] stopping sim"
docker stop leo_sim >/dev/null
echo "[diag] done"
