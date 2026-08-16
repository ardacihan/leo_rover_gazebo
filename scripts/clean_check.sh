#!/usr/bin/env bash
D=/ros2_ws/reports/collab_clean/office_coordinated
echo "=== clipped coverage (should stay <= ~372) ==="
tail -3 "$D/coverage.log" 2>/dev/null
echo "=== spread ==="
bash /ros2_ws/scripts/check_leo2_spread.sh "$D/traj.csv" 2>/dev/null
