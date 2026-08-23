#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
[[ $# -ge 2 ]] || fail "Usage: $0 WORKSPACE sim_leo1|real_root [DOOR_WIDTH_M] [LASER_FRAME] [CAMERA_FRAME]"
WORKSPACE="$(workspace_path "$1")"
PROFILE="$2"
DOOR_WIDTH="${3:-}"
source_workspace "$WORKSPACE"

case "$PROFILE" in
  sim_leo1)
    BASE=leo1/base_link; LASER="${4:-leo1/sensor_lidar_link}"; CAMERA="${5:-leo1/camera_link}"
    RAW_SCAN=/leo1/scan; FILTERED_SCAN=/leo1/scan_filtered
    CLOUD=/leo1/camera/points; ODOM=/leo1/odom ;;
  real_root)
    BASE=base_footprint; LASER="${4:-laser_frame}"; CAMERA="${5:-camera_link}"
    RAW_SCAN=/scan; FILTERED_SCAN=/scan_filtered
    CLOUD=/camera/camera/depth/color/points; ODOM=/wheel_odom ;;
  *) fail "Unknown profile: $PROFILE" ;;
esac

printf '=== Live topics ===\n'
printf '%-18s %-48s %s\n' "raw scan" "$RAW_SCAN" "$(ros2 topic type "$RAW_SCAN" 2>/dev/null || printf MISSING)"
printf '%-18s %-48s %s\n' "filtered scan" "$FILTERED_SCAN" "$(ros2 topic type "$FILTERED_SCAN" 2>/dev/null || printf MISSING)"
printf '%-18s %-48s %s\n' "camera cloud" "$CLOUD" "$(ros2 topic type "$CLOUD" 2>/dev/null || printf MISSING)"
printf '%-18s %-48s %s\n' "odometry" "$ODOM" "$(ros2 topic type "$ODOM" 2>/dev/null || printf MISSING)"
printf '\n=== Live TF values (read-only) ===\n'
ros2 run leo_nav2_exploration tf_snapshot "$BASE" "$LASER" || true
printf '\n'
ros2 run leo_nav2_exploration tf_snapshot "$BASE" "$CAMERA" || true
printf '\n=== Starting footprint ===\n'
FOOTPRINT_ARGS=(--front 0.21 --rear 0.21 --left 0.21 --right 0.21 --padding 0.01)
[[ -n "$DOOR_WIDTH" ]] && FOOTPRINT_ARGS+=(--door-width "$DOOR_WIDTH")
ros2 run leo_nav2_exploration footprint_tool "${FOOTPRINT_ARGS[@]}"
printf '\nUse the raw scan for LiDAR mounting calibration and the filtered scan for SLAM/navigation.\n'
printf 'Active calibration commands are documented in docs/CALIBRATION.md.\n'
