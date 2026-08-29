#!/usr/bin/env bash
# Record a rover-2 session that is actually replayable: maps, both costmaps,
# the scan slam_toolbox really consumes, the TF needed to align everything,
# and a camera stream that does not starve the graph.
#
#   ./record_session.sh <name> [duration_s]
#
# Writes to $BAGDIR/<name>/ (default ~/bags). Refuses to start if
# preflight_record.sh fails - pass SKIP_PREFLIGHT=1 to override, at your risk.
set -eo pipefail
cd "$(dirname "$0")"
source ./env.sh

NAME="${1:?usage: record_session.sh <name> [duration_s]}"
DUR="${2:-0}"
BAGDIR="${BAGDIR:-$HOME/bags}"
OUT="$BAGDIR/$NAME"

if [[ -e "$OUT" ]]; then echo "refusing to overwrite $OUT"; exit 1; fi

if [[ "${SKIP_PREFLIGHT:-0}" != "1" ]]; then
  ./preflight_record.sh "$BAGDIR" || {
    echo
    echo "preflight failed. Re-run with SKIP_PREFLIGHT=1 only if you accept a"
    echo "capture that may be unusable."
    exit 1
  }
fi

mkdir -p "$OUT"

# ---------------------------------------------------------------------------
# Topics. REQUIRED ones abort the recording if absent - a bag missing these is
# not worth the disk. OPTIONAL ones are recorded when present.
# ---------------------------------------------------------------------------
REQUIRED=(
  /tf /tf_static
  /scan /scan_filtered /scan_uniform      # /scan_uniform IS slam's input
  /map                                    # the SLAM map itself
  /global_costmap/costmap                 # map frame
  /local_costmap/costmap                  # odom frame - see NOTE below
  /wheel_odom /imu/data
  /firmware/imu /firmware/wheel_states /firmware/battery_averaged
  /rob_2/camera/color/camera_info
)
OPTIONAL=(
  /rob_2/camera/color/image_raw/compressed
  /rob_2/camera/extrinsics/depth_to_color
  /global_costmap/costmap_updates /local_costmap/costmap_updates
  /local_costmap/published_footprint
  /plan /local_plan
  /cmd_vel /cmd_vel_smoothed
  /joint_states
  /aruco/markers /aruco/registry
  /odom /odometry/filtered
)

# NOTE on frames: /local_costmap/costmap is published in `odom`,
# /global_costmap/costmap and /map in `map`. They only coincide while
# map->odom is identity. Recording /tf at full rate is what lets you put them
# in one frame offline - do not drop it.

live=$(ros2 topic list 2>/dev/null)
MISSING=()
for t in "${REQUIRED[@]}"; do grep -qx -- "$t" <<<"$live" || MISSING+=("$t"); done
if (( ${#MISSING[@]} )); then
  echo "ABORT - required topics are not advertised:"
  printf '   %s\n' "${MISSING[@]}"
  echo
  echo "If /scan_uniform is in that list, slam_toolbox has no input and this"
  echo "run would repeat 2026-08-25 run 3: identity map->odom, empty /map."
  exit 1
fi

TOPICS=("${REQUIRED[@]}")
for t in "${OPTIONAL[@]}"; do grep -qx -- "$t" <<<"$live" && TOPICS+=("$t"); done

if grep -qx -- /rob_2/camera/color/image_raw/compressed <<<"$live"; then
  echo "camera: recording COMPRESSED stream"
else
  echo "camera: no compressed stream - recording NO camera."
  echo "        raw image_raw is ~25 MB/s and stalls the recorder; start"
  echo "        image_transport republish first if you need video."
fi

QOS="$PWD/bag_qos.yaml"
[[ -f "$QOS" ]] || { echo "missing $QOS"; exit 1; }

echo
echo "recording ${#TOPICS[@]} topics -> $OUT/bag"
printf '   %s\n' "${TOPICS[@]}"
printf '%s\n' "${TOPICS[@]}" > "$OUT/topics.txt"
date -Is > "$OUT/started_at.txt"

# --max-cache-size buffers a disk hiccup instead of dropping messages.
# 512 MB is ~20 s of headroom at a sane (compressed) byte rate.
setsid nohup ros2 bag record \
  -o "$OUT/bag" \
  --qos-profile-overrides-path "$QOS" \
  --max-cache-size 536870912 \
  --storage-preset-profile resilient \
  "${TOPICS[@]}" \
  > "$OUT/record.log" 2>&1 < /dev/null &
BAGPID=$!
echo "$BAGPID" > "$OUT/record.pid"
echo "recorder pid $BAGPID   log: $OUT/record.log"

# --- live watchdog: the single number that would have caught every 2026-08-25
# --- failure. /map going stale while /cmd_vel is non-zero means stop driving.
(
  last_map_change=$(date +%s); last_hash=""
  while kill -0 "$BAGPID" 2>/dev/null; do
    h=$(timeout 3 ros2 topic echo /map --once --field info.map_load_time 2>/dev/null | md5sum | cut -c1-8)
    now=$(date +%s)
    if [[ -n "$h" && "$h" != "$last_hash" ]]; then last_hash="$h"; last_map_change=$now; fi
    age=$(( now - last_map_change ))
    if (( age > 15 )); then
      printf '\033[31m[%s] /map STALE for %ss - stop and restart SLAM\033[0m\n' \
        "$(date +%H:%M:%S)" "$age" | tee -a "$OUT/watchdog.log"
    fi
    sleep 5
  done
) &
echo "$!" > "$OUT/watchdog.pid"

if (( DUR > 0 )); then
  echo "recording for ${DUR}s..."
  sleep "$DUR"
  "$PWD/stop_record.sh" "$NAME"
else
  echo
  echo "drive now. stop with:  ./stop_record.sh $NAME"
fi
