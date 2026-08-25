#!/usr/bin/env bash
# RPLIDAR C1 bringup for rover 2 with rover-4-compatible frame names:
# scan frame_id = laser_frame, static TF base_footprint -> laser_frame
# (z=0.15, the jetson-02 calibration from start_slam_test.sh).
source "$(dirname "$0")/env.sh"
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.15 --yaw 0 \
  --frame-id base_footprint --child-frame-id laser_frame &
TF_PID=$!
trap 'kill $TF_PID 2>/dev/null' EXIT
while true; do
  ros2 run rplidar_ros rplidar_node --ros-args \
    -p channel_type:=serial \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser_frame \
    -p inverted:=false \
    -p angle_compensate:=true \
    -p scan_mode:=Standard
  echo "$(date +%H:%M:%S) rplidar exited ($?), respawning" >&2
  sleep 2
done
