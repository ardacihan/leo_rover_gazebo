#!/bin/bash
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

echo "=== run1 logic render"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_fixed/shadow_bag \
    /home/smirn/bags/drive_2026-08-20/metadata.yaml "$OUT/drive_2026-08-20/logic"
cp /home/smirn/replay/drive_2026-08-20_fixed/stack.log "$OUT/drive_2026-08-20/logic/stack.log"

echo "=== run2 logic render"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic"
cp /home/smirn/replay/drive_2026-08-20_run2/stack.log "$OUT/drive_2026-08-20_run2/logic/stack.log"

echo "=== dashboards"
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20" --title "Leo drive 2026-08-20 · run 1"
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20_run2" --title "Leo drive 2026-08-20 · run 2"
echo "=== done"
