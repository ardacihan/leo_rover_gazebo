#!/usr/bin/env bash
D=/ros2_ws/reports/collab_fast/office_coordinated
echo "=== coverage (sim-time reached) ==="
tail -2 "$D/coverage.log" 2>/dev/null
echo "=== both-robot spread ==="
bash /ros2_ws/scripts/check_leo2_spread.sh "$D/traj.csv" 2>/dev/null
echo "=== GPU util ==="
LD_LIBRARY_PATH=/usr/lib/wsl/lib nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader 2>/dev/null
