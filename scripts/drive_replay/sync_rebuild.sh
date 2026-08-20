#!/bin/bash
# Sync leo_nav2_exploration from the repo into the WSL workspace and rebuild.
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
rsync -a --exclude=__pycache__ --exclude=.pytest_cache \
    "$REPO/src/leo_nav2_exploration/" /home/smirn/leo_ws/src/leo_nav2_exploration/
source /opt/ros/humble/setup.bash
cd /home/smirn/leo_ws
PYTHONNOUSERSITE=1 colcon build --packages-select leo_nav2_exploration 2>&1 | tail -5
