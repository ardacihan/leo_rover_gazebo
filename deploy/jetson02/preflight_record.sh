#!/usr/bin/env bash
# Gate that must pass BEFORE recording. Every check here corresponds to a
# way the 2026-08-25 session produced an unusable capture.
#
#   exit 0 = safe to record        exit 1 = fix it first
#
# Usage:  ./preflight_record.sh [BAGDIR]
set -uo pipefail
cd "$(dirname "$0")"
source ./env.sh

BAGDIR="${1:-$HOME/bags}"
FAIL=0
ok()   { printf '  \033[32mOK  \033[0m %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; FAIL=1; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$1"; }

hz_of() {  # hz_of <topic> -> median rate on stdout, empty if silent
  timeout 12 ros2 topic hz "$1" --window 20 2>/dev/null \
    | grep -oP 'average rate: \K[0-9.]+' | tail -1
}
count_pub() { ros2 topic info "$1" 2>/dev/null | grep -oP 'Publisher count: \K[0-9]+'; }

echo "== 1. shared memory transport (env.sh must NOT force UDP-only)"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  bad "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE is set."
  echo "       This disables shared memory: every local node pushes over UDP"
  echo "       loopback and the recorder starves. Comment it out in env.sh."
else
  ok "shared memory available (no UDP-only profile forced)"
fi

echo "== 2. bag destination is fast storage"
if [[ ! -d "$BAGDIR" ]]; then
  warn "$BAGDIR does not exist yet, creating"; mkdir -p "$BAGDIR"
fi
SRC=$(df --output=source "$BAGDIR" | tail -1)
case "$SRC" in
  *mmcblk*) bad "$BAGDIR is on the SD card ($SRC). Write-back stalls will eat the run." ;;
  *nvme*|*sd[a-z]*) ok "$BAGDIR on $SRC" ;;
  *) warn "$BAGDIR on $SRC - confirm this is not the SD card" ;;
esac
FREE_GB=$(df -BG --output=avail "$BAGDIR" | tail -1 | tr -dc '0-9')
if (( FREE_GB < 20 )); then bad "only ${FREE_GB} GB free on $BAGDIR"; else ok "${FREE_GB} GB free"; fi

echo "== 3. clocks (SBC vs Jetson) - stale TF lookups come from here"
if command -v chronyc >/dev/null 2>&1; then
  OFF=$(chronyc tracking 2>/dev/null | grep -oP 'System time *: *\K[0-9.]+')
  if [[ -n "$OFF" ]]; then ok "chrony system time offset ${OFF}s"; else warn "chrony present but not tracking"; fi
else
  warn "chronyc not installed - cannot verify SBC/Jetson clock agreement"
fi

echo "== 4. sensors"
for t in /scan /wheel_odom /imu/data; do
  R=$(hz_of "$t")
  if [[ -z "$R" ]]; then bad "$t silent"; else ok "$t at ${R} Hz"; fi
done

echo "== 5. SLAM is actually fed and actually running"
# slam.yaml consumes /scan_uniform, published ONLY by the scan_normalizer in
# the navigation overlay. A teleop-only bringup leaves SLAM with no input at
# all: it comes up, publishes one identity map->odom, and nothing else ever.
R=$(hz_of /scan_uniform)
if [[ -z "$R" ]]; then
  bad "/scan_uniform silent - slam_toolbox has NO INPUT."
  echo "       scan_normalizer is not running. Start the navigation overlay,"
  echo "       or run it standalone:"
  echo "         ros2 run leo_nav2_exploration scan_normalizer --ros-args \\"
  echo "           -p input_topic:=/scan_filtered -p output_topic:=/scan_uniform"
else
  ok "/scan_uniform at ${R} Hz (slam_toolbox input)"
fi
R=$(hz_of /map)
if [[ -z "$R" ]]; then
  bad "/map silent - SLAM is not publishing a map (map_update_interval is 1.0 s)"
else
  ok "/map at ${R} Hz"
fi
P=$(count_pub /map)
[[ "${P:-0}" -ge 1 ]] && ok "/map has ${P} publisher(s)" || bad "/map has no publisher"

echo "== 6. map -> odom must MOVE once you drive"
echo "     drive the rover ~1 m now, then watch for a non-zero translation:"
timeout 10 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m2 -A1 "Translation" || warn "no map->odom yet"
echo "     (all-zero for a whole run means SLAM never scan-matched - see run3)"

echo "== 7. camera: compressed stream, NOT raw"
R=$(hz_of /rob_2/camera/color/image_raw/compressed)
if [[ -z "$R" ]]; then
  bad "compressed camera topic silent."
  echo "       The RealSense republishers advertise but never publish on this"
  echo "       rover. Start a real republisher before recording:"
  echo "         ros2 run image_transport republish raw compressed \\"
  echo "           --ros-args -r in:=/rob_2/camera/color/image_raw \\"
  echo "                      -r out/compressed:=/rob_2/camera/color/image_raw/compressed"
else
  ok "compressed camera at ${R} Hz (raw would be ~25 MB/s - never record it)"
fi

echo "== 8. no orphaned recorders"
N=$(pgrep -fc "bin/ros2 bag recor[d]" 2>/dev/null || echo 0)
if (( N > 0 )); then
  bad "$N recorder(s) still running - they will fight for disk and CPU"
  echo "       pkill -f 'bin/ros2 bag recor[d]'"
else
  ok "no recorder running"
fi

echo "== 9. local costmap is not a wall of inscribed cost"
echo "     inflation_radius 0.38 with a 0.26 m inscribed radius put 26-33% of"
echo "     the 4x4 m window at cost >= 99 on 2026-08-25, and all 44 goals aborted."
echo "     Check in open floor before trusting autonomy."

echo
if (( FAIL )); then
  printf '\033[31mPREFLIGHT FAILED - fix the items above before recording.\033[0m\n'
  exit 1
fi
printf '\033[32mPREFLIGHT PASSED - safe to record.\033[0m\n'
