#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
[[ $# -ge 3 ]] || fail "Usage: $0 WORKSPACE sim_leo1|real_root OUTPUT_BAG [--with-cloud]"
WORKSPACE="$(workspace_path "$1")"
PROFILE="$2"
OUTPUT="$3"
WITH_CLOUD=false
[[ "${4:-}" == "--with-cloud" ]] && WITH_CLOUD=true
source_workspace "$WORKSPACE"

case "$PROFILE" in
  sim_leo1)
    RAW_SCAN=/leo1/scan; FILTERED_SCAN=/leo1/scan_filtered; ODOM=/leo1/odom
    NAV=/leo1/cmd_vel_nav; SMOOTH=/leo1/cmd_vel_smoothed
    GUARDED=/leo1/cmd_vel_guarded; FINAL=/leo1/cmd_vel; CLOUD=/leo1/camera/points ;;
  real_root)
    RAW_SCAN=/scan; FILTERED_SCAN=/scan_filtered; ODOM=/wheel_odom
    NAV=/cmd_vel_nav; SMOOTH=/cmd_vel_smoothed
    GUARDED=/cmd_vel_guarded; FINAL=/cmd_vel; CLOUD=/camera/camera/depth/color/points ;;
  *) fail "Unknown profile: $PROFILE" ;;
esac
TOPICS=(
  "$RAW_SCAN" "$FILTERED_SCAN" "$ODOM" /tf /tf_static /map /map_metadata
  "$NAV" "$SMOOTH" "$GUARDED" "$FINAL"
  /plan /local_costmap/costmap_raw /global_costmap/costmap_raw
  /local_costmap/published_footprint
)
[[ "$WITH_CLOUD" == true ]] && TOPICS+=("$CLOUD")
exec ros2 bag record -o "$OUTPUT" "${TOPICS[@]}"
