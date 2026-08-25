#!/bin/bash
# Jog one rover through the top of the safety chain; print odom x before/after.
#   _jog_check.sh <ns>
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ns="${1:-leo1}"
b=$(timeout 15 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A2 'position:' | grep 'x:' | grep -oE '[-0-9.e]+')
(timeout 15 ros2 topic pub -r 5 /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{linear: {x: 0.15}}' >/dev/null 2>&1)
ros2 topic pub --once /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
a=$(timeout 15 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A2 'position:' | grep 'x:' | grep -oE '[-0-9.e]+')
echo "JOG $ns BEFORE=$b AFTER=$a"
