#!/usr/bin/env bash
# Summarize leo2's frontier goals and failures from an explorer log (arg 1).
L="${1:-/ros2_ws/reports/collab/office_world_coordinated/explorer.log}"
[ -f "$L" ] || { echo "no log: $L"; exit 0; }
echo "=== leo2 goal count ==="
grep -c 'leo2.frontier_explorer.*New frontier goal' "$L"
echo "=== leo2 goals with |y|>1.5 (room attempts) ==="
grep -oE 'leo2.frontier_explorer.*New frontier goal: \([-0-9.]+, [-0-9.]+\)' "$L" \
  | sed -E 's/.*\(([-0-9.]+), ([-0-9.]+)\)/\1 \2/' \
  | awk '($2>1.5 || $2<-1.5){print "  ("$1", "$2")"; c++} END{print "  room-goal count: "c+0}'
echo "=== leo2 failures ==="
echo "  No path:    $(grep -c 'leo2.*No path' "$L")"
echo "  goal failed:$(grep -c 'leo2.*goal failed' "$L")"
echo "  blacklisted:$(grep -c 'leo2.*blacklisted' "$L")"
echo "=== leo1 goals with |y|>1.5 for comparison ==="
grep -oE 'leo1.frontier_explorer.*New frontier goal: \([-0-9.]+, [-0-9.]+\)' "$L" \
  | sed -E 's/.*\(([-0-9.]+), ([-0-9.]+)\)/\1 \2/' \
  | awk '($2>1.5 || $2<-1.5){c++} END{print "  leo1 room-goal count: "c+0}'
