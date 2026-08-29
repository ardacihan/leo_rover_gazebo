#!/usr/bin/env bash
# Stop a recording started by record_session.sh and validate it immediately,
# while the rover is still in front of you and a re-run is cheap.
#
#   ./stop_record.sh <name>
set -uo pipefail
cd "$(dirname "$0")"
source ./env.sh

NAME="${1:?usage: stop_record.sh <name>}"
BAGDIR="${BAGDIR:-$HOME/bags}"
OUT="$BAGDIR/$NAME"
[[ -d "$OUT" ]] || { echo "no such run: $OUT"; exit 1; }

for p in watchdog record; do
  if [[ -f "$OUT/$p.pid" ]]; then
    kill "$(cat "$OUT/$p.pid")" 2>/dev/null || true
    rm -f "$OUT/$p.pid"
  fi
done

# `pkill -f rosbag2_recorder` does NOT match: that is the node name, not the
# process. Five orphans once spiked load to 34 and starved the firmware.
sleep 2
pkill -INT -f "bin/ros2 bag recor[d]" 2>/dev/null || true
sleep 3
LEFT=$(pgrep -fc "bin/ros2 bag recor[d]" 2>/dev/null || echo 0)
if (( LEFT > 0 )); then
  echo "warning: $LEFT recorder(s) still alive, sending TERM"
  pkill -f "bin/ros2 bag recor[d]" 2>/dev/null || true
fi
date -Is > "$OUT/stopped_at.txt"

# Save the SLAM map alongside the bag so the run is self-contained.
./save_map.sh "${NAME}_map" 2>/dev/null && cp -f maps/"${NAME}_map".{pgm,yaml} "$OUT/" 2>/dev/null || \
  echo "note: map_saver produced nothing (a VOLATILE /map cannot be saved by map_saver_cli)"

echo
echo "=== bag info ==="
ros2 bag info "$OUT/bag" 2>/dev/null | sed 's/^/  /'

echo
if [[ -f ../../scripts/check_bag.py ]]; then
  python3 ../../scripts/check_bag.py "$OUT/bag"
else
  echo "run scripts/check_bag.py on this bag before trusting it."
fi
