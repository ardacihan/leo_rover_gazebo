#!/usr/bin/env bash
# Close one leg of a run and leave it in the shape the offline merger expects.
#
#   tools/finish_run.sh <leg-name> [session-dir]
#
#   e.g.  tools/finish_run.sh legA
#         tools/finish_run.sh legB
#         # then, on the laptop:
#         python3 scripts/align_registries_offline.py ~/leo_runs/session1
#
# Run this ON the rover, while SLAM is still alive, at the end of each leg.
# Stop the bag first (Ctrl+C in its terminal); everything else can stay up.
#
# Why a script rather than three commands: `align_registries_offline.py` finds
# its inputs by globbing `aruco_registry_*.json` in one directory and looking
# for the matching map stem beside each. Get the names wrong and the merge
# reports "need two registries" with nothing else to say. This writes:
#
#   <session>/aruco_registry_<leg>.json    the confirmed landmark registry
#   <session>/aruco_detections_<leg>.csv   raw per-sighting samples
#   <session>/<leg>_map.pgm / .yaml        the saved occupancy grid
#   <session>/<leg>_posegraph.*            resumable slam_toolbox state
#   <session>/bags/<leg>_*/                whatever record_rover_bag.sh wrote
#
# Two legs driven with the SAME robot, SLAM restarted between them, are exactly
# two rovers with an unknown relative pose: each leg's map frame is anchored at
# that leg's own start. That is the point -- nothing about the merge knows or
# cares that it was one robot twice.
set -eo pipefail

LEG="${1:-}"
SESSION="${2:-$HOME/leo_runs/session1}"
[[ -n "$LEG" ]] || { echo "usage: $0 <leg-name> [session-dir]" >&2; exit 2; }

REGISTRY="${REGISTRY:-$HOME/leo_nav2_ws/runs/current/aruco_registry.json}"
SAMPLES="${SAMPLES:-$HOME/leo_nav2_ws/runs/current/aruco_detections.csv}"
BAGDIR="${BAGDIR:-$HOME/leo_bags}"

mkdir -p "$SESSION/bags"
echo "leg '$LEG' -> $SESSION"

# ---- map ----
# map_saver_cli subscribes TRANSIENT_LOCAL, which is what slam_toolbox offers,
# so the stock tool is correct here (unlike /shared_map in the sim stack).
if ros2 run nav2_map_server map_saver_cli -f "$SESSION/${LEG}_map" \
     --ros-args -p map_subscribe_transient_local:=true >/dev/null 2>&1; then
  echo "  map      -> ${LEG}_map.pgm/.yaml"
else
  echo "  WARNING: map_saver_cli failed -- is SLAM still running?" >&2
fi

# ---- pose graph ----
if ros2 service call /slam_toolbox/serialize_map \
     slam_toolbox/srv/SerializePoseGraph "{filename: $SESSION/${LEG}_posegraph}" \
     >/dev/null 2>&1; then
  echo "  posegraph-> ${LEG}_posegraph.*"
else
  echo "  note: pose graph not serialized (service absent)"
fi

# ---- ArUco registry: the file the merge actually needs ----
# The detector rewrites it every 5 s, so it is current as long as the detector
# ran. No detector, no registry, no merge -- and that failure is silent until
# the aligner says "need two registries".
if [[ -f "$REGISTRY" ]]; then
  cp "$REGISTRY" "$SESSION/aruco_registry_${LEG}.json"
  n=$(python3 -c "import json,sys;print(len(json.load(open(sys.argv[1])).get('markers',[])))" \
        "$SESSION/aruco_registry_${LEG}.json" 2>/dev/null || echo '?')
  echo "  registry -> aruco_registry_${LEG}.json  ($n confirmed markers)"
  if [[ "$n" =~ ^[0-9]+$ && "$n" -lt 2 ]]; then
    echo "  WARNING: fewer than 2 confirmed markers. The merge needs at least" >&2
    echo "           2 ids THIS leg shares with the other leg (3+ to spot a" >&2
    echo "           bad one with the leave-one-out table). Drive past more." >&2
  fi
else
  echo "  WARNING: no registry at $REGISTRY -- was aruco_detector running?" >&2
  echo "           Without it this leg cannot be merged with any other." >&2
fi
[[ -f "$SAMPLES" ]] && cp "$SAMPLES" "$SESSION/aruco_detections_${LEG}.csv" \
  && echo "  samples  -> aruco_detections_${LEG}.csv"

# ---- bag ----
latest=$(ls -1dt "$BAGDIR/${LEG}"_* 2>/dev/null | head -1 || true)
if [[ -n "$latest" ]]; then
  if [[ -f "$latest/metadata.yaml" ]]; then
    mv "$latest" "$SESSION/bags/"
    echo "  bag      -> bags/$(basename "$latest")"
  else
    echo "  WARNING: $latest has no metadata.yaml -- the recorder was killed" >&2
    echo "           rather than Ctrl+C'd. That bag will not play." >&2
  fi
else
  echo "  note: no bag named ${LEG}_* in $BAGDIR"
fi

echo
echo "leg '$LEG' closed. Registries in $SESSION:"
ls -1 "$SESSION"/aruco_registry_*.json 2>/dev/null | sed 's/^/    /' || echo '    (none)'
echo
echo "When two legs are done, on the laptop:"
echo "    scp -r <user>@<jetson>:$SESSION ."
echo "    python3 scripts/align_registries_offline.py $(basename "$SESSION") --refine"
