#!/usr/bin/env bash
D="${1:-/ros2_ws/reports/collab_big/husarion_single}"
echo "=== coverage ==="; tail -2 "$D/coverage.log" 2>/dev/null
echo "=== robot pose (single: leo1) ==="
timeout 4 ros2 run tf2_ros tf2_echo map leo1/base_link 2>/dev/null | grep -m1 -A1 Translation | tr '\n' ' '; echo
echo "=== explorer state ==="; grep -oE 'EXPLORING|SWEEPING|finished|New frontier goal' "$D/explorer.log" 2>/dev/null | tail -3
