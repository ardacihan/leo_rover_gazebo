#!/bin/bash
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2_tuned/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic"
cp /home/smirn/replay/drive_2026-08-20_run2_tuned/stack.log "$OUT/drive_2026-08-20_run2/logic/stack.log"

python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20_run2" 164 "$OUT/frames/ab_r2_t164_pipucks.png"
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20_run2" --title "Leo drive 2026-08-20 · run 2"

# refresh compact media for the tuned logic renders and rebuild embed pages
rm -rf "$OUT"/drive_2026-08-20*/compact
bash "$HERE/make_compact.sh"
bash "$HERE/make_artifact_pages.sh"
echo done
