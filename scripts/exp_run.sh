#!/usr/bin/env bash
# One autonomous-exploration experiment, for either navigation stack.
#
# Both stacks get the identical Gazebo world, sensor set (lidar + RGBD camera),
# odometry profile, recorders and wall-clock cap, so the artefacts are directly
# comparable.
#
# Usage:
#   exp_run.sh <stack> <world> <outdir> [profile] [cap_min]
#
#   stack:    orig    - slam_toolbox + nav2.launch.py + explore_lite (this repo)
#             bundle  - leo_nav2_exploration overlay + frontier_exploration_ros2
#   world:    husarion_office | office_world | depot_world | <path>.sdf
#   outdir:   repo-relative, created if missing
#   profile:  ideal      - Gazebo ground-truth odom TF (flatters SLAM)
#             realistic  - skid-steer wheel-odometry drift (what hardware gives)
#   cap_min:  wall-clock cap, default 35
#
# Artefacts: map.pgm/.yaml, timelapse.mp4 + _final.png, pose_error.csv,
# traj.csv, per-node logs, and the container's own launch log.
set -eo pipefail

STACK="$1"; WORLD="$2"; OUT="$3"; PROFILE="${4:-realistic}"; CAP_MIN="${5:-35}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$STACK" || -z "$WORLD" || -z "$OUT" ]]; then
  echo "usage: $0 <orig|bundle> <world> <outdir> [ideal|realistic] [cap_min]" >&2
  exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[exp $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }

in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

case "$PROFILE" in
  ideal)     GT_TF=true  ;;
  realistic) GT_TF=false ;;
  *) echo "unknown profile $PROFILE" >&2; exit 2 ;;
esac

LOG "stack=$STACK world=$WORLD profile=$PROFILE cap=${CAP_MIN}min out=$OUT"

# ---------- 1. simulator (lidar + RGBD camera, headless, GPU-passthrough) ----
WORLD="$WORLD" GUI=false ENABLE_CAMERA=true NUM_ROBOTS=1 GT_ODOM_TF="$GT_TF" \
  "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

LOG "waiting for /leo1/scan"
ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topic never appeared"; docker logs --tail 40 leo_sim; exit 1; }

SCAN_HZ=$(in_sim 'timeout 10 ros2 topic hz /leo1/scan 2>&1 | grep -m1 "average rate" || echo "none"')
LOG "scan: $SCAN_HZ"
sleep 3

# ---------- 2. odometry realism ----------
if [[ "$PROFILE" == "realistic" ]]; then
  if [[ "${USE_EKF:-0}" == "1" ]]; then
    # The wheel model still produces the drifting Odometry *message*, but the
    # EKF owns odom -> base_link instead, fusing wheel x-velocity with a
    # degraded gyro yaw rate. Two publishers of that TF would fight, so the
    # wheel model must not publish it.
    LOG "starting wheel-odometry model (message only) + gyro + EKF"
    in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
        -p use_sim_time:=true -p yaw_scale:=${YAW_SCALE:-0.12} \
        -p linear_scale:=${LINEAR_SCALE:-0.02} -p slip_per_metre:=${SLIP:-0.01} \
        -p seed:=${SEED:-1} -p publish_tf:=false > /ros2_ws/$OUT/odom.log 2>&1"
    in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args \
        -p use_sim_time:=true -p seed:=${SEED:-1} > /ros2_ws/$OUT/imu.log 2>&1"
    sleep 5
    in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args \
        --params-file /ros2_ws/scripts/ekf_leo.yaml \
        -r __node:=ekf_filter_node > /ros2_ws/$OUT/ekf.log 2>&1"
    sleep 8
  else
    LOG "starting skid-steer wheel-odometry model (replaces ground-truth TF)"
    in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
        -p use_sim_time:=true -p yaw_scale:=${YAW_SCALE:-0.12} \
        -p linear_scale:=${LINEAR_SCALE:-0.02} -p slip_per_metre:=${SLIP:-0.01} \
        -p seed:=${SEED:-1} > /ros2_ws/$OUT/odom.log 2>&1"
    sleep 8
  fi
fi

# ---------- 3. navigation stack ----------
case "$STACK" in
  orig)
    LOG "starting original stack: slam.launch.py + nav2.launch.py"
    in_sim_bg "exec ros2 launch leo_rover_gazebo slam.launch.py > /ros2_ws/$OUT/slam.log 2>&1"
    in_sim_bg "exec ros2 launch leo_rover_gazebo nav2.launch.py > /ros2_ws/$OUT/nav2.log 2>&1"
    READY_CHECK='ros2 action list 2>/dev/null | grep -q compute_path_to_pose'
    ;;
  bundle|hybrid)
    LOG "starting overlay stack: sim_navigation.launch.py (slam + nav2 + guard + collision monitor)"
    in_sim_bg "exec ros2 launch leo_nav2_exploration sim_navigation.launch.py \
        start_slam:=true enable_voxel:=${ENABLE_VOXEL:-true} autostart:=true \
        > /ros2_ws/$OUT/overlay.log 2>&1"
    READY_CHECK='ros2 action list 2>/dev/null | grep -q compute_path_to_pose'
    ;;
  *) LOG "FATAL: unknown stack $STACK"; exit 2 ;;
esac

LOG "waiting for Nav2 planner action"
ok=""
for _ in $(seq 1 48); do
  if in_sim "$READY_CHECK"; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up"; exit 1; }
sleep 10

# ---------- 3b. sensor evidence ----------
# The Gazebo rgbd_camera renders lazily: it emits nothing until something
# subscribes to its gz topic, and the ros_gz bridge only subscribes once a ROS
# subscriber exists. So the camera can only be measured *after* the stack that
# consumes it is running -- measuring earlier always reports zero and would
# make a "camera + lidar" claim look false when it is not.
CAM_HZ=$(in_sim 'timeout 15 ros2 topic hz /leo1/camera/points 2>&1 | grep -m1 "average rate" || echo "none"')
LOG "camera/points (with stack up): $CAM_HZ"
CAM_SUBS=$(in_sim 'ros2 topic info -v /leo1/camera/points 2>/dev/null | grep -c "Node name" || echo 0')
LOG "camera/points endpoints: $CAM_SUBS"
FILT_HZ=$(in_sim 'timeout 12 ros2 topic hz /leo1/scan_filtered 2>&1 | grep -m1 "average rate" || echo "n/a (orig stack has no scan_filtered)"')
LOG "scan_filtered: $FILT_HZ"

# ---------- 4. recorders ----------
LOG "starting recorders"
in_sim_bg "exec python3 /ros2_ws/scripts/pose_error_recorder.py /ros2_ws/$OUT/pose_error.csv 0.5 > /ros2_ws/$OUT/pose_rec.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/map_recorder.py /ros2_ws/$OUT/timelapse 10 > /ros2_ws/$OUT/timelapse.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py leo1 /ros2_ws/$OUT/traj.csv 1.0 > /ros2_ws/$OUT/traj.log 2>&1"
sleep 5

# ---------- 5. bootstrap jog ----------
# slam_toolbox publishes only the initial scan disk until the rover has moved
# ~minimum_travel_distance; the nearest frontier of that disk sits inside Nav2's
# goal tolerance, and explore_lite livelocks on an instantly-"reached" goal.
# Applied to both stacks so neither gets an unfair head start.
LOG "bootstrap jog"
in_sim 'timeout 10 ros2 topic pub -r 5 /leo1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" >/dev/null 2>&1 || true
ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 5

# ---------- 6. explorer ----------
case "$STACK" in
  orig)
    DONE_PATTERN='Exploration stopped\.'
    in_sim_bg "exec ros2 launch leo_rover_gazebo explore.launch.py > /ros2_ws/$OUT/explorer.log 2>&1"
    ;;
  hybrid)
    # frontier_exploration_ros2 declares "No more frontiers found" once the SLAM
    # map's free space reaches the edge of the occupancy grid -- it ended a run
    # at 24% coverage that way. explore_lite reads /map directly with a metric
    # min_frontier_size and does not have that failure mode (95.8% on the same
    # world), so pair it with the overlay's navigation and safety chain.
    DONE_PATTERN='Exploration stopped\.'
    in_sim_bg "exec ros2 launch leo_rover_gazebo explore.launch.py > /ros2_ws/$OUT/explorer.log 2>&1"
    ;;
  bundle)
    DONE_PATTERN='Exploration (complete|finished|stopped)'
    in_sim_bg "exec ros2 launch leo_nav2_exploration frontier_exploration.launch.py \
        > /ros2_ws/$OUT/explorer.log 2>&1"
    LOG "waiting for cold-idle /control_exploration service"
    svc=""
    for _ in $(seq 1 24); do
      if in_sim 'ros2 service list 2>/dev/null | grep -qE "(^|/)control_exploration$"'; then svc=1; break; fi
      sleep 5
    done
    if [[ -n "$svc" ]]; then
      LOG "enabling frontier exploration"
      in_sim 'ros2 run frontier_exploration_ros2 frontier_exploration_ctl start' \
        >>"$ROOT/$OUT/run.log" 2>&1 || LOG "WARNING: ctl start returned non-zero"
    else
      LOG "WARNING: control_exploration service never appeared"
    fi
    ;;
esac
LOG "explorer launched; polling (cap ${CAP_MIN} min)"

# ---------- 7. wait ----------
finished=""
deadline=$(( $(date +%s) + CAP_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died mid-run"; exit 1
  fi
  if grep -qE "$DONE_PATTERN" "$ROOT/$OUT/explorer.log" 2>/dev/null; then
    finished=1; LOG "explorer reports completion"; break
  fi
done
[[ -n "$finished" ]] || LOG "cap reached; collecting artefacts anyway"

in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 10

# ---------- 8. save ----------
LOG "saving map"
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/map \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed"

# ---------- 9. teardown ----------
LOG "flushing recorders"
in_sim 'pkill -INT -f "pose_error_recorder[.]py" || true; pkill -INT -f "map_recorder[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true' || true
sleep 20
docker stop leo_sim >/dev/null
LOG "done -> $OUT"
[[ -n "$finished" ]] || exit 3
