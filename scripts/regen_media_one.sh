#!/usr/bin/env bash
# Regenerate all media for ONE run dir (camera videos from the bag, map
# videos with markers, faststart remux). World is derived from the run name.
# Used by scripts/regen_all_media.sh with xargs -P for parallelism.
set -u
cd "$(dirname "$0")/.."
ROOT=$(pwd)
d=${1:?usage: regen_media_one.sh <run_dir>}
name=$(basename "$d")

case "$name" in
  *_l3_*)      WORLD=small_house_l3 ;;
  *_l9_*)      WORLD=small_house_l9 ;;
  *_l15_*)     WORLD=small_house_l15 ;;
  *_sh_*)      WORLD=small_house ;;
  *_office_*)  WORLD=office_world ;;
  *_depot_*)   WORLD=depot_world ;;
  *_husarion_*) WORLD=husarion_office ;;
  *)           WORLD=office_world ;;
esac

# Camera videos need rosbag2 -> container. Skip if already present.
if [ ! -f "$d/camera_leo1.mp4" ]; then
  docker run --rm -v "$ROOT":/ros2_ws leo_rover_humble:latest bash -lc \
    "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 scripts/extract_camera_video.py '$d'" \
    >/dev/null 2>&1 || echo "WARN camera $name"
fi
# Map videos (host).
if [ ! -f "$d/map_explore_leo1.mp4" ]; then
  python3 scripts/render_map_videos.py "$d" --world "$WORLD" \
    >/dev/null 2>&1 || echo "WARN mapvid $name"
fi
# Browser-friendly remux.
if [ -x tools/ffmpeg ]; then
  for v in "$d"/*.mp4; do
    [ -f "$v" ] || continue
    case "$v" in *".tmp.mp4") continue ;; esac
    tools/ffmpeg -y -loglevel error -i "$v" -c copy -movflags +faststart "$v.tmp.mp4" \
      >/dev/null 2>&1 && mv "$v.tmp.mp4" "$v" || rm -f "$v.tmp.mp4"
  done
fi
echo "done $name"
