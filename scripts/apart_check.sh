#!/usr/bin/env bash
D=/ros2_ws/reports/collab_clean/office_coordinated_apart
echo "=== coverage (clipped) ==="
tail -2 "$D/coverage.log" 2>/dev/null
echo "=== spread (divided?) ==="
bash /ros2_ws/scripts/check_leo2_spread.sh "$D/traj.csv" 2>/dev/null
