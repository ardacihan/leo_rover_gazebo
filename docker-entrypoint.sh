#!/usr/bin/env bash
set -e

# ── 1. ROS 2 Humble base ──────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 2. Leo description (source-built fallback) ────────────────────────────────
if [ -f /opt/leo_ws/install/setup.bash ]; then
    source /opt/leo_ws/install/setup.bash
fi

# ── 3. Project workspace (if already built) ───────────────────────────────────
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ── 4. Environment Variables ──────────────────────────────────────────────────
export GZ_VERSION=harmonic
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/ros2_ws/install/leo_rover_description/share"

# Default ROS Domain ID
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$@"