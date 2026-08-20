#!/bin/bash
# Copy the drive bags into WSL and run the default-view extraction for both.
set -e
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
mkdir -p ~/bags
for b in drive_2026-08-20 drive_2026-08-20_run2; do
  if [ ! -d ~/bags/$b ]; then
    cp -r $REPO/drive_2026-08-20/$b ~/bags/$b
  fi
done
for b in drive_2026-08-20 drive_2026-08-20_run2; do
  echo "=== extracting $b ==="
  python3 $REPO/scripts/drive_replay/extract_bag.py ~/bags/$b \
      $REPO/reports/drive_2026-08-20/$b/default
done
