#!/usr/bin/env bash
# Shared environment for the rover-2 (jetson-02) nav stack scripts.
source /opt/ros/humble/setup.bash
source "$HOME/leo_nav2_ws/install/setup.bash"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/leo_nav2_ws/fastdds_udp_only.xml
