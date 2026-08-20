#!/bin/bash
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
G=$REPO/scripts/drive_replay/grab_frames.py
O=$REPO/reports/drive_2026-08-20/frames
mkdir -p "$O"
R1=$REPO/reports/drive_2026-08-20/drive_2026-08-20
R2=$REPO/reports/drive_2026-08-20/drive_2026-08-20_run2
python3 "$G" "$R1" 472 "$O/r1_t472_shoes.png"
python3 "$G" "$R1" 502 "$O/r1_t502_lamp_human.png"
python3 "$G" "$R1" 510 "$O/r1_t510.png"
python3 "$G" "$R1" 300 "$O/r1_t300_parked_end.png"
python3 "$G" "$R2" 164 "$O/r2_t164_pipucks.png"
python3 "$G" "$R2" 200 "$O/r2_t200.png"
