#!/usr/bin/env bash
# Isolated ArUco detection test on the native-GPU WSL simulator.
#
# Ground-truth odometry plus a static identity map->leo1/odom means the `map`
# frame IS the world frame, so every metre of position error belongs to the
# detector -- corner noise, marker size, camera extrinsics -- and none of it to
# SLAM. On a full exploration run the same number is dominated by map drift and
# says nothing about whether the detector is right.
#
# Usage: scripts/aruco_test_wsl.sh <outdir> [route] [cap_min]

set -eo pipefail
OUT="${1:-reports/night/aruco_test}"
ROUTE="${2:-office_full}"
CAP_MIN="${3:-14}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DISTRO="${LEO_WSL_DISTRO:-Ubuntu}"
WS="${LEO_WSL_WS:-/home/smirn/leo_ws}"
WIN_ROOT="${LEO_WSL_WIN_ROOT:-/mnt/c/Users/smirn/Desktop/leo_rover_gazebo}"

mkdir -p "$ROOT/$OUT"
LOG() { echo "[aruco $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }

WSL_ENV="export PYTHONNOUSERSITE=1; source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; export GZ_VERSION=harmonic RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export GZ_SIM_RESOURCE_PATH=$WS/install/leo_rover_description/share:$WS/install/leo_description/share:$WS/src/husarion_gz_worlds/models:$WS/src/leo_rover_gazebo/models; export IGN_GAZEBO_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH; unset DISPLAY WAYLAND_DISPLAY; export XDG_RUNTIME_DIR=/tmp/runtime-smirn; mkdir -p \$XDG_RUNTIME_DIR 2>/dev/null; chmod 700 \$XDG_RUNTIME_DIR 2>/dev/null"

in_sim()    { wsl.exe -d "$DISTRO" -e bash -lc "$WSL_ENV; $1"; }
in_sim_bg() { wsl.exe -d "$DISTRO" -e bash -lc "$WSL_ENV; setsid nohup bash -c '$1' >/dev/null 2>&1 < /dev/null & disown; sleep 0.3"; }

# Nav2 servers and slam_toolbox are C++ binaries under /opt/ros/humble/lib;
# pkill on 'ros2' leaves them running and the next run inherits their state.
cleanup_sim() {
  # Standalone script, not an inline pkill: see scripts/leo_cleanup_wsl.sh for
  # why a pattern passed to `bash -c` makes the shell kill itself partway.
  local out
  out=$(wsl.exe -d "$DISTRO" -e bash -c "cp $WIN_ROOT/scripts/leo_cleanup_wsl.sh ~/leo_cleanup.sh && chmod +x ~/leo_cleanup.sh && ~/leo_cleanup.sh" 2>&1 | tail -1)
  LOG "$out"
  sleep 2
  case "$out" in
    *"cleanup: 0 ROS processes"*) ;;
    *) LOG "FATAL: ROS processes from a previous run survived -- refusing to start"
       exit 1 ;;
  esac
}

LOG "syncing and building"
in_sim "rsync -a --delete --exclude='__pycache__' $WIN_ROOT/src/ $WS/src/ && rsync -a --exclude='__pycache__' $WIN_ROOT/scripts/ $WS/scripts/" >>"$ROOT/$OUT/run.log" 2>&1
in_sim "cd $WS && colcon build --symlink-install --parallel-workers 8 --packages-select leo_nav2_exploration leo_rover_gazebo 2>&1 | tail -3" >>"$ROOT/$OUT/run.log" 2>&1

LOG "killing any previous sim"
cleanup_sim
sleep 3

LOG "starting simulator (GPU, ground-truth odom TF)"
in_sim_bg "cd $WS && ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:=office_world gui:=false num_robots:=1 enable_camera:=true gt_odom_tf:=true > $WIN_ROOT/$OUT/sim.log 2>&1"

ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/camera/image$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: camera topic never appeared"; exit 1; }
LOG "renderer: $(in_sim "grep -m1 GL_RENDERER ~/.ignition/rendering/ogre2.log 2>/dev/null || echo unknown")"
sleep 5

LOG "static map -> leo1/odom (identity; leo1/odom is world-anchored)"
in_sim_bg "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map leo1/odom --ros-args -p use_sim_time:=true > $WIN_ROOT/$OUT/static_tf.log 2>&1"
sleep 3

LOG "starting ArUco detector (marker_length=${ARUCO_LEN:-0.20})"
in_sim_bg "ros2 launch leo_nav2_exploration aruco.launch.py profile:=sim use_sim_time:=true marker_length:=${ARUCO_LEN:-0.20} max_range:=${ARUCO_MAX_RANGE:-6.0} min_hits:=${ARUCO_MIN_HITS:-3} registry_file:=$WIN_ROOT/$OUT/aruco_registry.json samples_file:=$WIN_ROOT/$OUT/aruco_samples.csv > $WIN_ROOT/$OUT/aruco.log 2>&1"
sleep 8
in_sim 'ros2 node list 2>/dev/null | grep -q aruco_detector' || { LOG "FATAL: aruco_detector did not start"; tail -30 "$ROOT/$OUT/aruco.log"; exit 1; }

LOG "driving route $ROUTE"
in_sim_bg "python3 $WS/scripts/scripted_drive.py --ros-args -p use_sim_time:=true -p route:=$ROUTE -p linear_speed:=0.30 -p angular_speed:=0.50 > $WIN_ROOT/$OUT/drive.log 2>&1"
in_sim_bg "python3 $WS/scripts/traj_recorder.py leo1 $WIN_ROOT/$OUT/traj.csv 1.0 > $WIN_ROOT/$OUT/traj.log 2>&1"

deadline=$(( $(date +%s) + CAP_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 30
  if grep -qiE "route complete|all waypoints" "$ROOT/$OUT/drive.log" 2>/dev/null; then LOG "route complete"; break; fi
done
in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 5

LOG "detector summary"
grep -E "CONFIRMED|frames=" "$ROOT/$OUT/aruco.log" | tail -12 | tee -a "$ROOT/$OUT/run.log" || true
cleanup_sim

LOG "scoring"
python3 "$ROOT/scripts/score_aruco.py" "$ROOT/$OUT/aruco_registry.json"   "$ROOT/src/leo_rover_exploration/config/mock_markers_office_world.yaml"   --json-out "$ROOT/$OUT/aruco_score.json"   --samples "$ROOT/$OUT/aruco_samples.csv"   --marker-length "${ARUCO_LEN:-0.20}" 2>&1 | tee -a "$ROOT/$OUT/run.log" || true
LOG "done -> $OUT"
