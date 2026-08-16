#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "=== /leo1/map topic info (pubs/subs + QoS) ==="
ros2 topic info /leo1/map -v 2>/dev/null | grep -iE 'Publisher count|Subscription count|Node name|Durability|Reliability' | head -30
echo
echo "=== does /leo1/map have data? (echo one, 4s) ==="
timeout 4 ros2 topic echo /leo1/map --field info.width --once 2>/dev/null || echo "NO DATA on /leo1/map"
echo
echo "=== frontier_explorer subscriptions ==="
ros2 node info /leo1/frontier_explorer 2>/dev/null | sed -n '/Subscribers/,/Publishers/p' | head -20
echo
echo "=== explorer log tail ==="
tail -8 /ros2_ws/reports/collab_ofdiag/office/explorer.log 2>/dev/null
