#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/smirn/leo_ws/install/setup.bash
cd /home/smirn/leo_ws/src/leo_nav2_exploration
PYTHONNOUSERSITE=1 python3 -m pytest test -q 2>&1 | tail -8
