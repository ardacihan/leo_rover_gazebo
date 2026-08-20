#!/bin/bash
# Build the self-contained compact dashboards (tuned + baseline variants)
# for publishing. Budget: embedded base64 must stay under the 16 MB
# artifact cap, so everything is 2 fps and depth stays local-only.
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20

python3 - <<'EOF'
import sys
sys.path.insert(0, '/mnt/c/Users/smirn/Desktop/leo_rover_gazebo/scripts/drive_replay')
from pathlib import Path
from compact_media import recode

# Artifact carries robust (logic/) + lidar-only; baseline and low-obstacle
# stay in the full-quality local dashboard only -- more than two variants
# of video do not fit the 16 MB artifact cap.
LC_W = {'drive_2026-08-20': 240, 'drive_2026-08-20_run2': 280}
base = Path('/mnt/c/Users/smirn/Desktop/leo_rover_gazebo/reports/drive_2026-08-20')
for run in ('drive_2026-08-20', 'drive_2026-08-20_run2'):
    PLAN = [
        ('default/color.mp4', 320, 4),
        ('default/lidar.mp4', 320, 4),
        ('logic/map.mp4', 320, 4),
        ('logic/global_costmap.mp4', 320, 4),
        ('logic/local_costmap.mp4', LC_W[run], 4),
        ('logic_lidar/map.mp4', 320, 4),
        ('logic_lidar/global_costmap.mp4', 320, 4),
        ('logic_lidar/local_costmap.mp4', LC_W[run], 4),
    ]
    total = 0
    for rel, width, drop in PLAN:
        src = base / run / rel
        if not src.exists():
            continue
        dst = base / run / 'compact' / rel
        recode(src, dst, width, drop)
        total += dst.stat().st_size
    print(f'{run}: compact total {total/1e6:.1f} MB raw '
          f'(~{total*1.334/1e6:.1f} MB embedded)')
EOF

python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20" \
    --title "Leo drive 2026-08-20 · run 1" --embed --compact
python3 "$HERE/build_drive_dashboard.py" "$OUT/drive_2026-08-20_run2" \
    --title "Leo drive 2026-08-20 · run 2" --embed --compact
ls -la "$OUT"/drive_2026-08-20*/dashboard_compact.html
