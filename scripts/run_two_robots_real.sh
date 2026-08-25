#!/usr/bin/env bash
# Validate two physical rover mapping graphs, then start only the shared
# alignment/map/dashboard layer. By default it never starts motion; explicit
# START_EXPLORERS=true also starts autonomous explorers through cmd_vel_nav.
#
# On rover 1 (after its firmware, lidar, RealSense and TF drivers):
#   ros2 launch leo_nav2_exploration real_mapping.launch.py \
#     robot_ns:=leo1 use_aruco:=true aruco_debug_image:=true \
#     marker_length:=0.15 allowed_ids:=1,2,3,4
#
# On rover 2, use the same command with robot_ns:=leo2. Both machines must use
# the same ROS_DOMAIN_ID and synchronized clocks. Then, on the operator laptop:
#   scripts/run_two_robots_real.sh [output-directory]
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${1:-reports/real_${STAMP}/two_robot_mapping}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
START_EXPLORERS="${START_EXPLORERS:-false}"
mkdir -p "$ROOT/$OUT"

LOG() { echo "[real two-rover $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }
fatal() { LOG "FATAL: $*"; exit 1; }

command -v ros2 >/dev/null 2>&1 || fatal "ros2 is not on PATH; source ROS and this workspace"
command -v curl >/dev/null 2>&1 || fatal "curl is required for the dashboard health check"

topic_has_sample() {
  timeout 8 ros2 topic echo --once "$1" --field header >/dev/null 2>&1
}

node_exists() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

param_value() {
  ros2 param get "$1" "$2" 2>/dev/null | sed -E 's/^[^:]+: //'
}

camera_frame() {
  timeout 8 ros2 topic echo --once \
    "/$1/camera/camera/color/image_raw" --field header.frame_id 2>/dev/null \
    | sed -n '/^---$/d; /^[[:space:]]*$/d; 1p'
}

LOG "STAGE 1/4: checking live physical sensor and SLAM topics"
for robot in leo1 leo2; do
  topic_has_sample "/$robot/scan" \
    || fatal "/$robot/scan is not delivering fresh lidar messages"
  topic_has_sample "/$robot/camera/camera/color/camera_info" \
    || fatal "/$robot RealSense camera_info is not delivering fresh messages"
  topic_has_sample "/$robot/camera/camera/color/image_raw" \
    || fatal "/$robot RealSense colour images are not delivering fresh messages"
  topic_has_sample "/$robot/map" \
    || fatal "/$robot/map is not delivering a SLAM map"
done

LOG "STAGE 2/4: auditing the real OpenCV ArUco detector contracts"
declare -A CAMERA_FRAMES
for robot in leo1 leo2; do
  detector="/$robot/aruco_detector"
  node_exists "$detector" \
    || fatal "$detector is absent; launch real_mapping with use_aruco:=true"

  image_topic="$(param_value "$detector" image_topic)"
  map_frame="$(param_value "$detector" map_frame)"
  optical="$(param_value "$detector" frame_is_optical)"
  max_range="$(param_value "$detector" max_range)"
  marker_length="$(param_value "$detector" marker_length)"
  dictionary="$(param_value "$detector" dictionary)"
  use_sim_time="$(param_value "$detector" use_sim_time)"
  optical="${optical,,}"
  use_sim_time="${use_sim_time,,}"
  expected_image="/$robot/camera/camera/color/image_raw"

  [[ "$image_topic" == "$expected_image" ]] \
    || fatal "$detector reads '$image_topic', expected '$expected_image'"
  [[ "$map_frame" == "$robot/map" ]] \
    || fatal "$detector publishes in '$map_frame', expected '$robot/map'"
  [[ "$optical" == "true" ]] \
    || fatal "$detector has frame_is_optical=$optical; RealSense requires true"
  [[ "$max_range" == "4.5" ]] \
    || fatal "$detector max_range=$max_range; field landmark gate must be 4.5 m"
  [[ "$use_sim_time" == "false" ]] \
    || fatal "$detector use_sim_time=$use_sim_time; physical runs require false"

  CAMERA_FRAMES[$robot]="$(camera_frame "$robot")"
  [[ -n "${CAMERA_FRAMES[$robot]}" ]] \
    || fatal "could not read /$robot RealSense optical frame_id"
  if ! timeout 8 ros2 run tf2_ros tf2_echo \
      "$robot/map" "${CAMERA_FRAMES[$robot]}" 2>/dev/null \
      | grep -q Translation; then
    fatal "no TF $robot/map <- ${CAMERA_FRAMES[$robot]}; ArUco pixels cannot be converted into $robot/map"
  fi

  publishers="$(ros2 topic info "/$robot/tag_detections" 2>/dev/null \
    | sed -nE 's/Publisher count: ([0-9]+)/\1/p')"
  [[ "${publishers:-0}" -ge 1 ]] \
    || fatal "/$robot/tag_detections has no publisher"
  registry_publishers="$(ros2 topic info "/$robot/aruco_markers" 2>/dev/null \
    | sed -nE 's/Publisher count: ([0-9]+)/\1/p')"
  [[ "${registry_publishers:-0}" -ge 1 ]] \
    || fatal "/$robot/aruco_markers has no persistent registry publisher; the aligner cannot reuse real observations after they leave the camera"
  LOG "  $robot PASS: real detector -> $map_frame from its own RealSense; dictionary=$dictionary black-square-length=${marker_length}m"
done

[[ "${CAMERA_FRAMES[leo1]}" != "${CAMERA_FRAMES[leo2]}" ]] \
  || fatal "both RealSense streams use frame '${CAMERA_FRAMES[leo1]}'; give the camera drivers unique frame prefixes before running two robots"

LOG "STAGE 3/4: starting hybrid tag+map estimation and vetted map merging"
ros2 launch multi_robot_shared_mapping shared_align.launch.py \
  use_sim_time:=false alignment_mode:=hybrid \
  enable_tag_alignment:=true enable_map_alignment:=true \
  enable_alignment_tf:=true compare_to_ground_truth:=false \
  >"$ROOT/$OUT/alignment.log" 2>&1 &
ALIGN_PID=$!

LOG "STAGE 4/4: starting camera/map/goal/execution dashboard"
python3 "$ROOT/scripts/live_multirobot_dashboard.py" \
  --output "$ROOT/$OUT" --port "$DASHBOARD_PORT" \
  >"$ROOT/$OUT/dashboard.log" 2>&1 &
DASH_PID=$!

EXPLORE_PID=""
if [[ "${START_EXPLORERS,,}" == "true" ]]; then
  LOG "OPTIONAL MOTION: starting coordinated explorers through each rover's cmd_vel_nav safety input"
  ros2 launch leo_rover_exploration collab_explore.launch.py \
    num_robots:=2 use_sim_time:=false coordination_mode:=coordinated \
    common_frame:=leo1/map shared_map_topic:=/shared_map_raw \
    command_topic_suffix:=cmd_vel_nav item_search:=false \
    >"$ROOT/$OUT/explorers.log" 2>&1 &
  EXPLORE_PID=$!
fi

cleanup() {
  LOG "stopping laptop alignment/dashboard processes (no rover process touched)"
  pids=("$DASH_PID" "$ALIGN_PID")
  [[ -n "$EXPLORE_PID" ]] && pids+=("$EXPLORE_PID")
  kill -INT "${pids[@]}" >/dev/null 2>&1 || true
  wait "${pids[@]}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ready=""
for _ in $(seq 1 20); do
  if curl --fail --silent "http://127.0.0.1:$DASHBOARD_PORT/api/status" \
      >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 1
done
[[ -n "$ready" ]] || fatal "dashboard did not answer; see $ROOT/$OUT/dashboard.log"

LOG "READY: http://127.0.0.1:$DASHBOARD_PORT/"
LOG "Watch the execution stage. No map or peer position is trusted before ALIGNMENT LOCKED."
LOG "Exact first-estimate and first-peer-use times are printed in alignment.log, rover explorer logs, and telemetry.jsonl."
if [[ -n "$EXPLORE_PID" ]]; then
  LOG "AUTONOMY ACTIVE: explorers use /leo1/cmd_vel_nav and /leo2/cmd_vel_nav; shared-union completion stops them in place."
else
  LOG "Alignment/dashboard only. After validating both Nav2 and safety chains, rerun with START_EXPLORERS=true to enable autonomous motion."
fi

pids=("$ALIGN_PID" "$DASH_PID")
[[ -n "$EXPLORE_PID" ]] && pids+=("$EXPLORE_PID")
wait -n "${pids[@]}"
