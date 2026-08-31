#!/usr/bin/env bash
# Post-process every run under $BASE (default final/runs; for a bundle pass
# BASE=final/bundles/<id>/runs) and rebuild that bundle dashboard.
set -u
cd "$(dirname "$0")/.."
ROOT=$(pwd)
BASE=${BASE:-final/runs}
WORLD=${WORLD:-office_world}
TITLE=${TITLE:-}
NOTE=${NOTE:-}

for d in "$BASE"/run*; do
  [ -d "$d" ] || continue
  echo "== $d =="
  # FAST_FINALIZE=1 (overnight suites): skip video rendering - bags and
  # timelapse snapshots keep the raw data, so videos can be produced later
  # by re-running this script without the flag.
  if [ -z "${FAST_FINALIZE:-}" ]; then
    # Full camera videos from the bag (needs rosbag2 -> container).
    docker run --rm -v "$ROOT":/ros2_ws leo_rover_humble:latest bash -lc \
      "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 scripts/extract_camera_video.py '$d'" \
      || echo "WARN: camera extraction failed for $d"
    # Per-robot + merged exploration map videos (host).
    python3 scripts/render_map_videos.py "$d" --world "$WORLD" || echo "WARN: map videos failed for $d"
  fi
  # Remux every video with the index at the front (+faststart) so browsers
  # start playback immediately from the file system.
  if [ -x tools/ffmpeg ]; then
    for v in "$d"/*.mp4; do
      [ -f "$v" ] || continue
      tools/ffmpeg -y -loglevel error -i "$v" -c copy -movflags +faststart "$v.tmp.mp4" \
        && mv "$v.tmp.mp4" "$v" || rm -f "$v.tmp.mp4"
    done
  fi
  # Markers must be glued FLUSH on walls: fit each authored marker's yaw
  # against the wall direction in this run's own merged map. Perpendicular
  # or floating plates (husarion ids 1/4/7, found 2026-08-30) are invisible
  # to the cameras and quietly ruin marker scores.
  SPAWN=$(python3 -c "import sys; sys.path.insert(0,'src/leo_rover_gazebo/launch');
from spawn_poses import SPAWN_POSES
p = SPAWN_POSES.get('$WORLD', {}).get('leo1')
print(f'{p[0]} {p[1]} {p[5]}' if p else '')" 2>/dev/null)
  if [ -n "$SPAWN" ] && [ -f "$d/merged_map.yaml" ]; then
    read -r sx sy syaw <<<"$SPAWN"
    python3 scripts/check_marker_orientation.py \
      "src/leo_rover_exploration/config/mock_markers_$WORLD.yaml" \
      "$d/merged_map.yaml" --spawn-x "$sx" --spawn-y "$sy" --spawn-yaw "$syaw" \
      > "$d/marker_orientation.txt" 2>&1 \
      || echo "WARN: markers not flush on walls (see $d/marker_orientation.txt)"
  fi
  # Room-completeness probes (its 20 corners are authored for office_world).
  if [ "$WORLD" = "office_world" ]; then
    python3 scripts/validate_office_run.py "$d" > "$d/office_validation.host.json" 2>"$d/validate.log" \
      || echo "NOTE: office validation flagged $d (see $d/validate.log)"
    if [ -s "$d/office_validation.host.json" ] && [ ! -f "$d/office_validation.json" ]; then
      cp "$d/office_validation.host.json" "$d/office_validation.json"
    fi
  fi
done

# Snap-packaged browsers refuse root-owned files under $HOME, so everything
# the containers wrote must belong to the user or images/videos silently
# show as broken.
docker run --rm -v "$ROOT":/ros2_ws leo_rover_humble:latest \
  bash -c "chown -R $(id -u):$(id -g) /ros2_ws/$(dirname "$BASE")"

python3 scripts/build_final_dashboard.py --base "$(dirname "$BASE")" \
  --world "$WORLD" ${TITLE:+--title "$TITLE"} ${NOTE:+--note "$NOTE"}
echo "dashboard: $(dirname "$BASE")/index.html"
