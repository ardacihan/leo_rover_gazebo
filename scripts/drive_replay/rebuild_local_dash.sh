#!/bin/bash
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
python3 "$REPO/scripts/drive_replay/build_drive_dashboard.py" \
    "$REPO/reports/drive_2026-08-20/drive_2026-08-20" --title "Leo drive 2026-08-20 · run 1"
python3 "$REPO/scripts/drive_replay/build_drive_dashboard.py" \
    "$REPO/reports/drive_2026-08-20/drive_2026-08-20_run2" --title "Leo drive 2026-08-20 · run 2"
