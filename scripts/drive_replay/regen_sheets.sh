#!/bin/bash
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay
OUT=$REPO/reports/drive_2026-08-20
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 472 "$OUT/frames/robust_r1_t472.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20" 502 "$OUT/frames/robust_r1_t502.png"
python3 "$HERE/cmp_ab.py" "$OUT/drive_2026-08-20_run2" 164 "$OUT/frames/robust_r2_t164.png"
