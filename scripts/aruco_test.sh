#!/usr/bin/env bash
# Isolated ArUco detection test -- detector accuracy without SLAM in the way.
#
# Runs the simulator with ground-truth odometry and a *static* identity
# map->odom transform, so the `map` frame is exactly the world frame. Any
# position error in the scored result is then the detector's own error: corner
# noise, pose ambiguity, camera extrinsics. On a full exploration run the same
# number is dominated by SLAM drift and says nothing about the detector.
#
# Usage: scripts/aruco_test.sh <outdir> [route] [cap_min]

set -eo pipefail
OUT="${1:-reports/night/aruco_test}"
ROUTE="${2:-office_full}"
CAP_MIN="${3:-12}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mkdir -p "$ROOT/$OUT"
LOG() { echo "[aruco $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }
in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

LOG "starting simulator (office_world, ground-truth odom TF)"
WORLD=office_world GUI=false ENABLE_CAMERA=true NUM_ROBOTS=1 GT_ODOM_TF=true \
  "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/camera/image$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: camera topic never appeared"; exit 1; }
sleep 5

# map == world. leo1/odom is world-anchored (see the odom investigation), so
# an identity map->leo1/odom makes map-frame detections directly comparable to
# the world-frame ground truth.
LOG "static map -> leo1/odom"
in_sim_bg "exec ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map leo1/odom \
    --ros-args -p use_sim_time:=true > /ros2_ws/$OUT/static_tf.log 2>&1"
sleep 3

LOG "starting ArUco detector"
in_sim_bg "exec ros2 launch leo_nav2_exploration aruco.launch.py profile:=sim \
    use_sim_time:=true marker_length:=${ARUCO_LEN:-0.20} \
    max_range:=${ARUCO_MAX_RANGE:-6.0} min_hits:=${ARUCO_MIN_HITS:-3} \
    registry_file:=/ros2_ws/$OUT/aruco_registry.json \
    > /ros2_ws/$OUT/aruco.log 2>&1"
sleep 8
if ! in_sim 'ros2 node list 2>/dev/null | grep -q aruco_detector'; then
  LOG "FATAL: aruco_detector did not start"; tail -40 "$ROOT/$OUT/aruco.log"; exit 1
fi

LOG "driving route $ROUTE"
in_sim_bg "exec python3 /ros2_ws/scripts/scripted_drive.py --ros-args \
    -p use_sim_time:=true -p route:=$ROUTE -p linear_speed:=0.30 \
    -p angular_speed:=0.50 > /ros2_ws/$OUT/drive.log 2>&1"

deadline=$(( $(date +%s) + CAP_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 30
  if grep -q "route complete\|Route complete\|finished" "$ROOT/$OUT/drive.log" 2>/dev/null; then
    LOG "route complete"; break
  fi
done
sleep 5
in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'

LOG "detector summary"
grep -E "CONFIRMED|frames=" "$ROOT/$OUT/aruco.log" | tail -20 | tee -a "$ROOT/$OUT/run.log" || true

docker stop leo_sim >/dev/null 2>&1 || true
LOG "scoring"
python3 "$ROOT/scripts/score_aruco.py" "$ROOT/$OUT/aruco_registry.json" \
  "$ROOT/src/leo_rover_exploration/config/mock_markers_office_world.yaml" \
  --json-out "$ROOT/$OUT/aruco_score.json" 2>&1 | tee -a "$ROOT/$OUT/run.log" || true
LOG "done -> $OUT"
