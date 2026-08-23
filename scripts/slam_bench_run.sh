#!/usr/bin/env bash
# One SLAM benchmark run: fixed route, one sensor/odometry profile, one
# slam_toolbox configuration. Motion comes from scripted_drive.py (closed on
# ground-truth pose), so every configuration is scored on the same trajectory.
#
# Usage:
#   slam_bench_run.sh <name> <profile> <slam_cfg> <outdir> [world] [route] [cap_min]
#
#   profile:   ideal      - Gazebo ground-truth odom TF, clean 20 m lidar
#                           (what this repo has always measured)
#              realistic  - wheel-odometry drift + noisy 12 m lidar + camera
#                           self-return (what the physical rover supplies)
#              miscal     - realistic, plus an uncalibrated lidar mount
#   slam_cfg:  path (container-side) to a slam_toolbox params yaml
#   outdir:    repo-relative output directory
#
# Artefacts: map.pgm/.yaml, pose_error.csv, slam.log, odom.log, drive.log.
set -eo pipefail

NAME="$1"; PROFILE="$2"; CFG="$3"; OUT="$4"
WORLD="${5:-office_world}"; ROUTE="${6:-office_full}"; CAP_MIN="${7:-60}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$NAME" || -z "$PROFILE" || -z "$CFG" || -z "$OUT" ]]; then
  echo "usage: $0 <name> <profile> <slam_cfg> <outdir> [world] [route] [cap_min]" >&2
  exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[bench $(date +%H:%M:%S)] $*"; }

in_sim() { docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

case "$PROFILE" in
  ideal)     GT_TF=true;  SCAN=/leo1/scan ;;
  realistic|miscal) GT_TF=false; SCAN=/leo1/scan_real ;;
  *) echo "unknown profile $PROFILE" >&2; exit 2 ;;
esac
# Lets a run feed SLAM the self-filtered scan instead of the raw one.
SCAN="${SCAN_TOPIC:-$SCAN}"

# ---------- 1. sim ----------
LOG "run '$NAME': profile=$PROFILE world=$WORLD route=$ROUTE cfg=$CFG"
WORLD="$WORLD" GUI=false ENABLE_CAMERA=false NUM_ROBOTS=1 GT_ODOM_TF="$GT_TF" \
  "$ROOT/scripts/sim_gpu_wsl.sh" >/dev/null

LOG "waiting for /leo1/scan"
ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topic never appeared"; docker logs --tail 40 leo_sim; exit 1; }
sleep 5

# ---------- 2. realism layer ----------
if [[ "$PROFILE" != "ideal" ]]; then
  LOG "starting realism layer (wheel odometry + degraded lidar)"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
      -p use_sim_time:=true -p yaw_scale:=${YAW_SCALE:-0.12} \
      -p linear_scale:=${LINEAR_SCALE:-0.02} -p slip_per_metre:=${SLIP:-0.01} \
      -p seed:=${SEED:-1} > /ros2_ws/$OUT/odom.log 2>&1"

  SCAN_FRAME=""
  if [[ "$PROFILE" == "miscal" ]]; then
    # The stack believes the lidar sits exactly on the URDF joint; publish the
    # scan in a frame that is deliberately offset from it. An x/y mount error
    # sweeps the scan sideways during in-place turns, which is what doubles
    # walls in a room map.
    SCAN_FRAME="-p output_frame:=leo1/lidar_assumed_link"
    in_sim_bg "exec ros2 run tf2_ros static_transform_publisher \
        ${MISCAL_X:-0.03} ${MISCAL_Y:--0.03} 0 ${MISCAL_YAW:-0.035} 0 0 \
        leo1/sensor_lidar_link leo1/lidar_assumed_link --ros-args \
        -p use_sim_time:=true"
  fi
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_scan.py --ros-args \
      -p use_sim_time:=true -p range_noise:=${RANGE_NOISE:-0.02} \
      -p range_max:=${RANGE_MAX:-12.0} -p dropout_rate:=${DROPOUT:-0.02} \
      -p self_return:=${SELF_RETURN:-true} -p seed:=${SEED:-1} \
      $SCAN_FRAME > /ros2_ws/$OUT/scan.log 2>&1"
  sleep 8
fi

# ---------- 3. slam ----------
LOG "starting slam_toolbox on $SCAN"
in_sim_bg "exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file $CFG -p use_sim_time:=true -p scan_topic:=$SCAN \
    -r /scan:=$SCAN > /ros2_ws/$OUT/slam.log 2>&1"

LOG "waiting for /map"
ok=""
for _ in $(seq 1 24); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/map$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: slam_toolbox never published /map"; tail -30 "$ROOT/$OUT/slam.log"; exit 1; }
sleep 5

# ---------- 4. recorders ----------
in_sim_bg "exec python3 /ros2_ws/scripts/pose_error_recorder.py \
    /ros2_ws/$OUT/pose_error.csv 1.0 > /ros2_ws/$OUT/recorder.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/map_recorder.py \
    /ros2_ws/$OUT/timelapse 20 > /ros2_ws/$OUT/timelapse.log 2>&1"

# ---------- 5. drive ----------
LOG "driving route (cap ${CAP_MIN} min)"
set +e
timeout $((CAP_MIN * 60)) docker exec leo_sim bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   python3 /ros2_ws/scripts/scripted_drive.py --ros-args -p use_sim_time:=true \
     -p route:=$ROUTE -p linear_speed:=${LINEAR_SPEED:-0.30} \
     -p angular_speed:=${ANGULAR_SPEED:-0.50}" \
  > "$ROOT/$OUT/drive.log" 2>&1
drive_rc=$?
set -e
if [[ $drive_rc -eq 124 ]]; then
  LOG "WARNING: drive hit the ${CAP_MIN} min cap"
else
  LOG "drive finished (rc=$drive_rc)"
fi
in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 10

# ---------- 6. save ----------
LOG "saving map"
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/map \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed"

# ---------- 7. teardown ----------
in_sim 'pkill -INT -f "pose_error_recorder[.]py" || true; pkill -INT -f "map_recorder[.]py" || true' || true
sleep 8
docker stop leo_sim >/dev/null
LOG "done -> $OUT"
