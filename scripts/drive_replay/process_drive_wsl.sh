#!/bin/bash
# Full pipeline for one real-rover drive bag: default view, stack shadow
# replay, and the combined dashboard.
#
#   wsl -d Ubuntu -- bash scripts/drive_replay/process_drive_wsl.sh <bag_name>
#
# <bag_name> is a rosbag2 directory under drive_2026-08-20/ in the repo (or an
# absolute WSL path to any bag directory). Outputs land in
# reports/drive_2026-08-20/<bag_name>/{default,logic,dashboard.html}.
set -e
BAGNAME=${1:?bag name or path}
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
HERE=$REPO/scripts/drive_replay

if [ -d "$BAGNAME" ]; then
  BAG=$BAGNAME
  NAME=$(basename "$BAGNAME")
else
  NAME=$BAGNAME
  mkdir -p ~/bags
  # bags live on the NTFS side; copy once for sqlite speed
  [ -d ~/bags/"$NAME" ] || cp -r "$REPO/drive_2026-08-20/$NAME" ~/bags/"$NAME"
  BAG=~/bags/$NAME
fi
OUT=$REPO/reports/drive_2026-08-20/$NAME

echo "==== 1/4 default-view extraction"
python3 "$HERE/extract_bag.py" "$BAG" "$OUT/default"

echo "==== 2/4 shadow replay through the rover stack (real-time)"
bash "$HERE/replay_drive_wsl.sh" "$BAG" ~/replay/"$NAME"

echo "==== 3/4 render replay outputs"
python3 "$HERE/extract_replay.py" ~/replay/"$NAME"/shadow_bag \
    "$BAG/metadata.yaml" "$OUT/logic"
cp ~/replay/"$NAME"/stack.log "$OUT/logic/stack.log" 2>/dev/null || true

echo "==== 4/4 dashboard"
python3 "$HERE/build_drive_dashboard.py" "$OUT" --title "Leo drive $NAME"
echo "open $OUT/dashboard.html"
