#!/usr/bin/env bash
# Respawn wrapper: laser_filters dies sporadically with int64_t overflow.
source "$(dirname "$0")/env.sh"
while true; do
  ros2 run laser_filters scan_to_scan_filter_chain --ros-args \
    -r __node:=scan_to_scan_filter_chain \
    --params-file "$HOME/leo_nav2_ws/install/leo_nav2_exploration/share/leo_nav2_exploration/config/real/scan_filter.yaml" \
    -r scan:=/scan -r scan_filtered:=/scan_filtered
  echo "$(date +%H:%M:%S) scan filter exited ($?), respawning" >&2
  sleep 2
done
