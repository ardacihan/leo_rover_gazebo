#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/smirn/leo_ws/install/setup.bash
ros2 pkg list | grep -E 'explore_lite|leo_nav2_exploration|nav2_bringup|laser_filters|robot_localization'
ros2 pkg prefix leo_nav2_exploration
python3 -c "import cv2; fourcc=cv2.VideoWriter_fourcc(*'avc1'); w=cv2.VideoWriter('/tmp/t.mp4', fourcc, 5, (64,64)); print('avc1', w.isOpened()); w.release()"
python3 -c "import cv2; fourcc=cv2.VideoWriter_fourcc(*'VP80'); w=cv2.VideoWriter('/tmp/t.webm', fourcc, 5, (64,64)); print('vp80', w.isOpened()); w.release()"
