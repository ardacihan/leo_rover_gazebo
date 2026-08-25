#!/usr/bin/env bash
# Read-only health checks for rover 2. Run before start_stack.sh and again
# before start_explore.sh.
cd "$(dirname "$0")"
source ./env.sh
echo "== battery (want > 10.3 V)"
timeout 8 ros2 topic echo /firmware/battery_averaged --once 2>/dev/null | grep data || echo MISSING
echo "== /scan (want ~10 Hz)"
timeout 12 ros2 topic hz /scan --window 10 2>&1 | head -1
echo "== /wheel_odom (want ~20 Hz, needs odom relay)"
timeout 12 ros2 topic hz /wheel_odom --window 10 2>&1 | head -1
echo "== odom -> base_footprint TF"
timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 "Translation" || echo MISSING
echo "== base_footprint -> laser_frame TF (needs run_lidar)"
timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_frame 2>&1 | grep -m1 "Translation" || echo MISSING
echo "== camera pointcloud (want ~25 Hz)"
timeout 12 ros2 topic hz /rob_2/camera/depth/color/points --window 10 2>&1 | head -1
echo "== /scan_filtered (needs stack or run_scanfilter)"
timeout 12 ros2 topic hz /scan_filtered --window 10 2>&1 | head -1
echo "== /cmd_vel publishers (want exactly 1: collision_monitor, once stack up)"
timeout 8 ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" || echo MISSING
echo "== competing nav services (all should be inactive except leo-hardware)"
systemctl is-active leo-hardware leo-ros leo-nav 2>/dev/null | tr '\n' ' '; echo "(order: leo-hardware leo-ros leo-nav)"
