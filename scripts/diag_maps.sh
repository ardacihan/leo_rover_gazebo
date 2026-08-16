#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "=== slam log tail ==="
tail -14 /ros2_ws/reports/collab_ofdiag/office/slam.log 2>/dev/null
echo "=== map-ish topics ==="
ros2 topic list 2>/dev/null | grep map
echo "=== /map publishers ==="
ros2 topic info /map 2>/dev/null
