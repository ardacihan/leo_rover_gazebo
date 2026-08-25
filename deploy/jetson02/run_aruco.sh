#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
mkdir -p "$HOME/leo_nav2_ws/runs/rover2_live_1"
exec ros2 run leo_nav2_exploration aruco_detector --ros-args \
  -p image_topic:=/rob_2/camera/color/image_raw \
  -p camera_info_topic:=/rob_2/camera/color/camera_info \
  -p dictionary:=DICT_4X4_50 \
  -p marker_length:=0.08 \
  -p allowed_ids:="[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]" \
  -p frame_is_optical:=true \
  -p rate_limit_hz:=5.0 \
  -p registry_file:="$HOME/leo_nav2_ws/runs/rover2_live_1/aruco_registry.json" \
  -p samples_file:="$HOME/leo_nav2_ws/runs/rover2_live_1/aruco_detections.csv"
