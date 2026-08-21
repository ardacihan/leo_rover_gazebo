#!/usr/bin/env bash
# Start SLAM + Nav2 + safety chain. The robot does NOT move from this alone.
# Mirror of ~/leo_nav2_ws/start_stack.sh on jetson-4 (2026-08-21 state).
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source "$HOME/leo_nav2_ws/install/setup.bash"
export ROS_DOMAIN_ID=4
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/leo_nav2_ws/fastdds_udp_only.xml
mkdir -p logs maps

# leo-nav.service crash-loops at boot (package gone from ros_ws) and would
# own a competing /cmd_vel publisher if it ever came up. Account password
# equals username on the lab rovers.
if systemctl is-active --quiet leo-nav; then
  echo "jetson-04" | sudo -S systemctl stop leo-nav && echo "stopped leo-nav"
fi

# Boot hygiene: these come back on every reboot and each costs ~a core.
pkill -f 'leo_real_experiments.*color_detecto[r]' && echo "killed color_detector" || true
pkill -f 'leo_real_experiments.*exploration_superviso[r]' && echo "killed exploration_supervisor" || true
pkill -f 'stuck_recover[y]' && echo "killed stuck_recovery" || true

# The Jetson RealSense build ships the NEON pointcloud filter and the boot
# configuration leaves it off; costmap camera obstacles need it. Decimation 4
# keeps the cloud small enough that cloud_filter's TF thread stays fresh.
timeout 20 ros2 param set /rob_4/camera pointcloud__neon_.enable true \
  || echo "WARN: could not enable camera pointcloud (is leo-ros up?)"
timeout 20 ros2 param set /rob_4/camera decimation_filter.enable true \
  || echo "WARN: could not enable decimation filter"
timeout 20 ros2 param set /rob_4/camera decimation_filter.filter_magnitude 4 \
  || echo "WARN: could not set decimation magnitude"

if [[ -f logs/stack.pid ]] && kill -0 "$(cat logs/stack.pid)" 2>/dev/null; then
  echo "stack already running (pid $(cat logs/stack.pid)); stop_all.sh first"
  exit 1
fi
setsid nohup ros2 launch leo_nav2_exploration real_navigation.launch.py navigation_start_delay:=25.0 \
  > logs/stack.log 2>&1 &
echo $! > logs/stack.pid
echo "stack starting (pgid $(cat logs/stack.pid)); log: ~/leo_nav2_ws/logs/stack.log"
