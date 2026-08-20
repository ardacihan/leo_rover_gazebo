#!/bin/bash
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

echo "=== render tuned run1"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_tuned/shadow_bag \
    /home/smirn/bags/drive_2026-08-20/metadata.yaml "$OUT/drive_2026-08-20/logic"
cp /home/smirn/replay/drive_2026-08-20_tuned/stack.log "$OUT/drive_2026-08-20/logic/stack.log"

echo "=== render tuned run2"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2_tuned/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic"
cp /home/smirn/replay/drive_2026-08-20_run2_tuned/stack.log "$OUT/drive_2026-08-20_run2/logic/stack.log"

echo "=== A/B sheets"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 472 "$OUT/frames/ab_r1_t472_shoes.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 502 "$OUT/frames/ab_r1_t502_lamp_human.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20_run2" 164 "$OUT/frames/ab_r2_t164_pipucks.png"

echo "=== dashboards"
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20" --title "Leo drive 2026-08-20 · run 1"
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20_run2" --title "Leo drive 2026-08-20 · run 2"
echo done
