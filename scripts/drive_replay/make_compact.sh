#!/bin/bash
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
for r in drive_2026-08-20 drive_2026-08-20_run2; do
  python3 "$REPO/scripts/drive_replay/compact_media.py" \
      "$REPO/reports/drive_2026-08-20/$r" --width 360 --drop 2
done
