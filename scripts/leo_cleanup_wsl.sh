#!/usr/bin/env bash
# Kill every simulator and ROS node from a previous run.
#
# Written as a *file* rather than passed to `bash -c` on purpose: a pkill
# pattern handed to a shell appears in that shell's own /proc/*/cmdline, so the
# shell matches its own pattern and dies partway through the cleanup. Here the
# invoking command line is just this script's path.
#
# Killing 'ros2' and 'python3' is also not enough: every Nav2 server and
# slam_toolbox is a C++ binary under /opt/ros/humble/lib. Leaving them running
# makes the next run inherit the previous run's finished map, which shows up as
# "No frontiers found, stopping" two seconds after launch.
PATTERNS=(
  'ign gazebo'
  'gz sim'
  '/opt/ros/humble/lib'
  '/opt/ros/humble/bin/ros2'
  '/home/smirn/leo_ws/install'
  'sim_realism'
  '_recorder.py'
  'scripted_drive'
)
self=$$
for pass in 1 2; do
  pids=""
  for pat in "${PATTERNS[@]}"; do
    pids="$pids $(pgrep -f "$pat" 2>/dev/null)"
  done
  for p in $pids; do
    [ "$p" = "$self" ] && continue
    [ "$p" = "$PPID" ] && continue
    kill -9 "$p" 2>/dev/null
  done
  sleep 2
done
left=$(ps -eo args --no-headers | grep -E '/opt/ros/|leo_ws/install|ign gazebo|gz sim' | grep -v grep | wc -l)
echo "cleanup: $left ROS processes still alive"
