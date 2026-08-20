#!/bin/bash
# Re-render the baseline-profile shadow bags with the current palette so the
# dashboard's A/B columns look identical apart from the data.
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_fixed/shadow_bag \
    /home/smirn/bags/drive_2026-08-20/metadata.yaml "$OUT/drive_2026-08-20/logic_baseline"
python3 "$HERE/extract_replay.py" /home/smirn/replay/drive_2026-08-20_run2/shadow_bag \
    /home/smirn/bags/drive_2026-08-20_run2/metadata.yaml "$OUT/drive_2026-08-20_run2/logic_baseline"
echo done
