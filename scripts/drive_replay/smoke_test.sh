#!/bin/bash
# 60 s smoke test of the replay stack against the start of the main bag.
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
OUT=/tmp/replay_smoke
source /opt/ros/humble/setup.bash
source /home/smirn/leo_ws/install/setup.bash
rm -rf "$OUT"; mkdir -p "$OUT"
bash "$REPO/scripts/leo_cleanup_wsl.sh"

CFG=$(mktemp -d /tmp/drive_replay_cfg.XXXX)
ros2 launch "$HERE/replay_stack.launch.py" config_dir:="$CFG" > "$OUT/stack.log" 2>&1 &
python3 "$HERE/depth_to_points.py" --ros-args -p use_sim_time:=true > "$OUT/depth.log" 2>&1 &
sleep 8
timeout --signal=INT 75 ros2 bag play ~/bags/drive_2026-08-20 --clock 50 > "$OUT/play.log" 2>&1
sleep 3
echo "--- topics with sim time still up:"
timeout 10 ros2 topic list | grep -E 'map|costmap|plan|shadow|points'
echo "--- /map info:"
timeout 12 ros2 topic echo /map --once --field info 2>&1 | head -8
echo "--- /global_costmap/costmap:"
timeout 12 ros2 topic echo /global_costmap/costmap --once --field info 2>&1 | head -8
echo "--- /cmd_vel_shadow count over 5s (clock stopped so likely 0):"
timeout 6 ros2 topic hz /cmd_vel_shadow 2>&1 | tail -2
bash "$REPO/scripts/leo_cleanup_wsl.sh"
echo "--- stack.log errors:"
grep -iE 'error|exception|died|failed' "$OUT/stack.log" | grep -v "log-level" | head -20
echo "--- stack.log tail:"
tail -25 "$OUT/stack.log"
echo "--- depth log tail:"
tail -3 "$OUT/depth.log"
rm -rf "$CFG"
