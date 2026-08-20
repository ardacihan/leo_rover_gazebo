#!/bin/bash
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

echo "=== render lidar-only run1"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_lidar/shadow_bag \
    /home/smirn/bags/drive_2026-08-20/metadata.yaml "$OUT/drive_2026-08-20/logic_lidar"
cp /home/smirn/replay/drive_2026-08-20_lidar/stack.log "$OUT/drive_2026-08-20/logic_lidar/stack.log"

echo "=== render lidar-only run2"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2_lidar/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic_lidar"
cp /home/smirn/replay/drive_2026-08-20_run2_lidar/stack.log "$OUT/drive_2026-08-20_run2/logic_lidar/stack.log"

echo "=== sheets: robust vs lidar-only"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 472 "$OUT/frames/lidar_r1_t472.png" \
    logic:robust logic_lidar:lidar-only
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 502 "$OUT/frames/lidar_r1_t502.png" \
    logic:robust logic_lidar:lidar-only
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20_run2" 164 "$OUT/frames/lidar_r2_t164.png" \
    logic:robust logic_lidar:lidar-only

echo "=== dashboards"
bash "$HERE/rebuild_local_dash.sh"
rm -rf "$OUT"/drive_2026-08-20*/compact
bash "$HERE/make_artifact_pages.sh"
echo done
