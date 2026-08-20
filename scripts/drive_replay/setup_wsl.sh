#!/bin/bash
# One-time WSL setup for the drive-bag dashboard pipeline.
set -u
pip3 install --quiet rosbags 2>&1 | tail -2
python3 -c 'import rosbags; print("rosbags ok")'
if ! command -v ffmpeg >/dev/null; then
  sudo -n apt-get install -y ffmpeg >/dev/null 2>&1 || echo "no-sudo: ffmpeg not installed (cv2 VideoWriter fallback will be used)"
fi
command -v ffmpeg >/dev/null && echo "ffmpeg ok" || echo "no-ffmpeg"
source /home/smirn/leo_ws/install/setup.bash 2>/dev/null || true
ros2 pkg list 2>/dev/null | grep -E 'explore_lite|leo_nav2_exploration|nav2_bringup' || true
