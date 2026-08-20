#!/bin/bash
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

echo "=== verify run2"
bash "$HERE/verify_shadow.sh" /home/smirn/replay/drive_2026-08-20_run2_robust | head -4

echo "=== render robust run1"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_robust/shadow_bag \
    /home/smirn/bags/drive_2026-08-20/metadata.yaml "$OUT/drive_2026-08-20/logic"
cp /home/smirn/replay/drive_2026-08-20_robust/stack.log "$OUT/drive_2026-08-20/logic/stack.log"

echo "=== render robust run2"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2_robust/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic"
cp /home/smirn/replay/drive_2026-08-20_run2_robust/stack.log "$OUT/drive_2026-08-20_run2/logic/stack.log"

echo "=== A/B sheets (baseline vs robust)"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 472 "$OUT/frames/robust_r1_t472.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 502 "$OUT/frames/robust_r1_t502.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20_run2" 164 "$OUT/frames/robust_r2_t164.png"

echo "=== dashboards"
bash "$HERE/rebuild_local_dash.sh"
rm -rf "$OUT"/drive_2026-08-20*/compact
bash "$HERE/make_artifact_pages.sh"
echo done
