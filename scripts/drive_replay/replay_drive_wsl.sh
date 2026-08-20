#!/bin/bash
# Shadow-replay a real-rover drive bag through the leo_nav2_exploration stack.
#
#   bash replay_drive_wsl.sh <bag_dir_in_wsl> <out_dir>
#
# Plays the bag on its own clock while the exact real-rover node set (scan
# filter, slam_toolbox, Nav2, velocity guard, collision monitor, explore_lite,
# plus a depth->PointCloud2 bridge for the camera costmap layer) runs against
# it, and records everything the stack produced into <out_dir>/shadow_bag.
BAG=${1:?bag dir}
OUT=${2:?output dir}
PROFILE=${3:-real}   # or real_baseline_2026-08-20 for the frozen snapshot
LIDAR_ONLY=${4:-false}   # true = drop the camera costmap layer (lidar only)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo

source /opt/ros/humble/setup.bash
source /home/smirn/leo_ws/install/setup.bash
export RCUTILS_LOGGING_BUFFERED_STREAM=1

mkdir -p "$OUT"
rm -rf "$OUT/shadow_bag"

bash "$REPO/scripts/leo_cleanup_wsl.sh"

CFG=$(mktemp -d /tmp/drive_replay_cfg.XXXX)
echo "=== launching replay stack (configs in $CFG)"
ros2 launch "$HERE/replay_stack.launch.py" config_dir:="$CFG" \
    config_profile:="$PROFILE" lidar_only:="$LIDAR_ONLY" > "$OUT/stack.log" 2>&1 &
LAUNCH_PID=$!

python3 "$HERE/depth_to_points.py" --ros-args -p use_sim_time:=true \
    > "$OUT/depth_to_points.log" 2>&1 &
DEPTH_PID=$!

sleep 8

# The Nav2 bringup occasionally loses a change_state service response
# (rmw timeout) and the lifecycle manager then hangs at "Configuring ..."
# forever -- the whole replay runs with no costmaps and no planner. Gate on
# the planner's costmap actually existing before wasting a real-time pass;
# one relaunch usually clears it.
for attempt in 1 2; do
  for i in $(seq 1 30); do
    if timeout 5 ros2 topic list 2>/dev/null | grep -q '^/global_costmap/costmap$'; then
      break 2
    fi
    sleep 2
  done
  if [ "$attempt" = 1 ]; then
    echo "=== Nav2 stuck in bringup; relaunching stack"
    kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 5
    bash "$REPO/scripts/leo_cleanup_wsl.sh"
    ros2 launch "$HERE/replay_stack.launch.py" config_dir:="$CFG" \
        config_profile:="$PROFILE" lidar_only:="$LIDAR_ONLY" \
        > "$OUT/stack.log" 2>&1 &
    LAUNCH_PID=$!
    python3 "$HERE/depth_to_points.py" --ros-args -p use_sim_time:=true \
        > "$OUT/depth_to_points.log" 2>&1 &
    DEPTH_PID=$!
    sleep 8
  else
    echo "=== Nav2 never came up; aborting" ; exit 1
  fi
done

echo "=== recording shadow bag"
ros2 bag record --use-sim-time -o "$OUT/shadow_bag" \
    /map /scan_filtered /plan \
    /local_costmap/costmap /global_costmap/costmap \
    /local_costmap/published_footprint \
    /cmd_vel /cmd_vel_shadow /cmd_vel_guarded /cmd_vel_nav \
    /explore/frontiers /merged_odom /tf /tf_static /rosout \
    > "$OUT/record.log" 2>&1 &
RECORD_PID=$!

sleep 3
echo "=== playing $BAG"
ros2 bag play "$BAG" --clock 50 > "$OUT/play.log" 2>&1
echo "=== play finished"
sleep 5

kill -INT "$RECORD_PID" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
kill "$DEPTH_PID" 2>/dev/null
sleep 8
kill -9 "$RECORD_PID" 2>/dev/null
bash "$REPO/scripts/leo_cleanup_wsl.sh"
rm -rf "$CFG"

echo "=== shadow bag:"
ls -la "$OUT/shadow_bag" 2>/dev/null
grep -cE 'Sending goal|frontier' "$OUT/stack.log" 2>/dev/null | \
    sed 's/^/goal-ish log lines: /'
echo done
