#!/usr/bin/env bash
# Detect ArUco markers from a recorded bag, after the fact.
#
#   tools/offline_aruco.sh <bag-dir> <leg-name> [session-dir]
#
#   e.g. tools/offline_aruco.sh session1/bags/legA_20260828_1012 legA session1
#        tools/offline_aruco.sh session1/bags/legB_20260828_1105 legB session1
#        python3 scripts/align_registries_offline.py session1 --refine
#
# Runs anywhere ROS 2 Humble and the leo_nav2_exploration package are
# available -- laptop under WSL is fine, nothing needs the robot.
#
# Why you would rather do this than run the detector in the lab:
#
#   * `marker_length` is the one parameter that can be wrong without anything
#     erroring -- the pose just lands short or long along the view ray, and the
#     merge inherits the error. Offline you measure the plates properly, re-run,
#     and compare. In the lab you get one guess.
#   * Same for `dictionary` and `allowed_ids`. A wrong dictionary detects
#     nothing and looks exactly like "no markers in view".
#   * It is one fewer thing to start, watch and restart between legs -- and
#     forgetting to restart the detector between legs corrupts the registry
#     (leg A's markers in leg A's frame, leg B's in leg B's, one file).
#
# What it needs from the bag, all of which record_rover_bag.sh records:
#   /bag/color/compressed     the pictures
#   /bag/color/camera_info    K and D -- without these there is no pose
#   /tf, /tf_static           map <- camera at each frame's stamp; the map->odom
#                             half comes from the SLAM that ran live, so marker
#                             poses land in the same frame as the saved map
#
# The one thing you cannot recover afterwards: frames you never recorded.
# At the throttle's 2 Hz and the detector's min_hits=3, a marker must be in
# view for ~1.5 s to be confirmed. Dwell on them.
set -eo pipefail

BAG="${1:-}"
LEG="${2:-}"
SESSION="${3:-.}"
[[ -n "$BAG" && -n "$LEG" ]] || {
  echo "usage: $0 <bag-dir> <leg-name> [session-dir]" >&2; exit 2; }
[[ -f "$BAG/metadata.yaml" ]] || {
  echo "FATAL: $BAG has no metadata.yaml -- not a playable bag" >&2; exit 1; }

MARKER_LENGTH="${MARKER_LENGTH:-0.08}"
DICTIONARY="${DICTIONARY:-DICT_4X4_50}"
ALLOWED_IDS="${ALLOWED_IDS:-[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]}"
RATE="${RATE:-1.0}"
MAP_FRAME="${MAP_FRAME:-map}"

mkdir -p "$SESSION"
REG="$SESSION/aruco_registry_${LEG}.json"
CSV="$SESSION/aruco_detections_${LEG}.csv"

echo "bag           $BAG"
echo "marker_length $MARKER_LENGTH m   dictionary $DICTIONARY"
echo "registry ->   $REG"
echo

cleanup() { kill $DEC_PID $DET_PID 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# use_sim_time everywhere and --clock on the player: the detector looks TF up
# at each image's stamp, and on wall-clock those stamps are minutes in the past,
# so every lookup fails and the registry comes out empty.
python3 "$(dirname "$0")/decompress_color.py" --ros-args \
  -p use_sim_time:=true > /tmp/decompress_$LEG.log 2>&1 &
DEC_PID=$!

ros2 run leo_nav2_exploration aruco_detector --ros-args \
  -p use_sim_time:=true \
  -p image_topic:=/bag/color/image_raw \
  -p camera_info_topic:=/bag/color/camera_info \
  -p map_frame:="$MAP_FRAME" \
  -p dictionary:="$DICTIONARY" \
  -p marker_length:="$MARKER_LENGTH" \
  -p allowed_ids:="$ALLOWED_IDS" \
  -p frame_is_optical:=true \
  -p publish_tf:=false \
  -p registry_file:="$REG" \
  -p samples_file:="$CSV" > "/tmp/aruco_$LEG.log" 2>&1 &
DET_PID=$!

sleep 4
echo "playing (rate $RATE)..."
ros2 bag play "$BAG" --clock --rate "$RATE"

# The registry is rewritten on a 5 s timer, so give the last one time to land.
sleep 7
cleanup

echo
if [[ -f "$REG" ]]; then
  n=$(python3 -c "import json,sys;print(len(json.load(open(sys.argv[1])).get('markers',[])))" "$REG")
  echo "$REG: $n confirmed markers"
  python3 -c "
import json,sys
for m in json.load(open(sys.argv[1])).get('markers', []):
    print('  id %-3d  (%.2f, %.2f)  hits=%d' % (m['id'], m['x'], m['y'], m.get('hits', 0)))
" "$REG"
  if [[ "$n" -lt 2 ]]; then
    echo
    echo "Fewer than 2 markers. Before assuming they were not in view, check:"
    echo "  * dictionary  -- a wrong one detects nothing at all"
    echo "  * marker_length / allowed_ids"
    echo "  * /tmp/aruco_$LEG.log for 'no TF map <- <frame>' (bag has no map->odom)"
    echo "  * /tmp/decompress_$LEG.log for zero frames (bag has no colour)"
  fi
else
  echo "no registry written -- see /tmp/aruco_$LEG.log"
  exit 1
fi
