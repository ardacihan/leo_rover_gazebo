#!/usr/bin/env bash
# Shared environment for the rover-2 (jetson-02) nav stack scripts.
source /opt/ros/humble/setup.bash
source "$HOME/leo_nav2_ws/install/setup.bash"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=0

# DO NOT re-enable. Forcing the UDP-only profile disables shared memory, so
# all ~25 Jetson-local nodes push their traffic over UDP loopback instead.
# That starved the micro-ROS agent (552 "session hard timeout" in 10 min,
# killing /firmware/*) and is the standing suspect for the recorder losing
# 50% of run 3 on 2026-08-25. preflight_record.sh fails if it is set.
# export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/leo_nav2_ws/fastdds_udp_only.xml
