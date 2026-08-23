#!/usr/bin/env bash
# exp_run.sh, but the simulator runs in the native WSL Ubuntu distro instead of
# Docker Desktop.
#
# Why: Docker Desktop ships no NVIDIA EGL/GLX ICD and no `d3d12_dri.so`, so
# Ogre always lands on `swrast` there. The native distro has both, and the same
# launch reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)` in
# `~/.ignition/rendering/ogre2.log`. Measured on office_world, headless, camera
# on: real-time factor 1.34 vs 1.15 and gz-server CPU ~194% vs ~320%. The RTF
# gain is modest; the ~40% CPU it gives back to Nav2 is the point, because the
# planner and controller were both missing their rates under the Docker stack.
#
# Everything downstream of the simulator is identical to exp_run.sh -- the same
# realism nodes, the same overlay, the same recorders, the same artefacts -- so
# runs from the two backends are comparable except for the CPU budget.
#
# Usage:  exp_run_wsl.sh <orig|bundle|hybrid> <world> <outdir> [profile] [cap_min]
# Env:    USE_EKF=1  USE_ARUCO=1  SEED=n  ARUCO_LEN=  YAW_SCALE=  LINEAR_SCALE=

set -eo pipefail

STACK="$1"; WORLD="$2"; OUT="$3"; PROFILE="${4:-realistic}"; CAP_MIN="${5:-35}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DISTRO="${LEO_WSL_DISTRO:-Ubuntu}"
WS="${LEO_WSL_WS:-/home/smirn/leo_ws}"
# Artefacts are written straight to the Windows-side repo through /mnt/c, so
# there is nothing to copy back and the scoring tools read the same paths as a
# Docker run.
WIN_ROOT="${LEO_WSL_WIN_ROOT:-/mnt/c/Users/smirn/Desktop/leo_rover_gazebo}"

if [[ -z "$STACK" || -z "$WORLD" || -z "$OUT" ]]; then
  echo "usage: $0 <orig|bundle|hybrid> <world> <outdir> [ideal|realistic] [cap_min]" >&2
  exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[wsl-exp $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }

# `bash -lc` in this distro does not source ROS, and the user's pip --user
# `empy` 4.1 shadows apt's 3.3.4 and breaks every rosidl import, so
# PYTHONNOUSERSITE is not optional here.
WSL_ENV="export PYTHONNOUSERSITE=1; source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; export GZ_VERSION=harmonic RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export GZ_SIM_RESOURCE_PATH=$WS/install/leo_rover_description/share:$WS/install/leo_description/share:$WS/src/husarion_gz_worlds/models:$WS/src/leo_rover_gazebo/models; export IGN_GAZEBO_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH; unset DISPLAY WAYLAND_DISPLAY; export XDG_RUNTIME_DIR=/tmp/runtime-smirn; mkdir -p \$XDG_RUNTIME_DIR 2>/dev/null; chmod 700 \$XDG_RUNTIME_DIR 2>/dev/null"

in_sim()    { wsl.exe -d "$DISTRO" -e bash -lc "$WSL_ENV; $1"; }
in_sim_bg() { wsl.exe -d "$DISTRO" -e bash -lc "$WSL_ENV; setsid nohup bash -c '$1' >/dev/null 2>&1 < /dev/null & disown; sleep 0.3"; }

# Killing 'ros2' and 'python3' is not enough and the gap is silent: every Nav2
# server and slam_toolbox is a C++ binary under /opt/ros/humble/lib, so they
# survive, keep publishing /map and TF, and the next run inherits the previous
# run's finished map -- which reads as "No frontiers found, stopping" two
# seconds after launch. Match the install trees instead of the launcher.
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

case "$PROFILE" in
  ideal)     GT_TF=true  ;;
  realistic) GT_TF=false ;;
  *) echo "unknown profile $PROFILE" >&2; exit 2 ;;
esac

LOG "backend=wsl:$DISTRO stack=$STACK world=$WORLD profile=$PROFILE cap=${CAP_MIN}min out=$OUT"

# ---------- 0. sync sources ----------
# The WSL workspace is a copy; config edits happen on the Windows side. Sync
# the two directories that carry them, then rebuild only if a package's Python
# or config changed (symlink-install means edits to installed files are live,
# but new entry points and new launch files are not).
LOG "syncing src/ and scripts/ into $WS"
in_sim "rsync -a --delete --exclude='__pycache__' $WIN_ROOT/src/ $WS/src/ && rsync -a --exclude='__pycache__' $WIN_ROOT/scripts/ $WS/scripts/" \
  >>"$ROOT/$OUT/run.log" 2>&1
if [[ "${REBUILD:-1}" == "1" ]]; then
  LOG "colcon build (leo_nav2_exploration, leo_rover_gazebo)"
  in_sim "cd $WS && colcon build --symlink-install --parallel-workers 8 \
      --packages-select leo_nav2_exploration leo_rover_gazebo 2>&1 | tail -3" \
    >>"$ROOT/$OUT/run.log" 2>&1 || { LOG "FATAL: colcon build failed"; exit 1; }
fi

# ---------- 1. simulator ----------
LOG "killing any previous sim"
cleanup_sim
sleep 3

LOG "starting simulator (GPU, headless)"
in_sim_bg "cd $WS && ros2 launch leo_rover_gazebo two_robots_gpu.launch.py \
    world:=$WORLD gui:=false num_robots:=1 enable_camera:=true gt_odom_tf:=$GT_TF \
    > $WIN_ROOT/$OUT/sim.log 2>&1"

LOG "waiting for /leo1/scan"
ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topic never appeared"; tail -30 "$ROOT/$OUT/sim.log" 2>/dev/null; exit 1; }

SCAN_HZ=$(in_sim 'timeout 10 ros2 topic hz /leo1/scan 2>&1 | grep -m1 "average rate" || echo none')
LOG "scan: $SCAN_HZ"

# Renderer evidence, recorded per run so a GPU claim is never taken on faith.
RENDERER=$(in_sim "grep -m1 'GL_RENDERER' ~/.ignition/rendering/ogre2.log 2>/dev/null || grep -m1 'GL_RENDERER' ~/.gz/rendering/ogre2.log 2>/dev/null || echo unknown")
LOG "renderer: $RENDERER"
sleep 3

# ---------- 2. odometry realism ----------
if [[ "$PROFILE" == "realistic" ]]; then
  if [[ "${USE_EKF:-0}" == "1" ]]; then
    LOG "wheel-odometry model (message only) + gyro + EKF"
    in_sim_bg "python3 $WS/scripts/sim_realism_odom.py --ros-args \
        -p use_sim_time:=true -p yaw_scale:=${YAW_SCALE:-0.12} \
        -p linear_scale:=${LINEAR_SCALE:-0.02} -p slip_per_metre:=${SLIP:-0.01} \
        -p seed:=${SEED:-1} -p publish_tf:=false > $WIN_ROOT/$OUT/odom.log 2>&1"
    in_sim_bg "python3 $WS/scripts/sim_realism_imu.py --ros-args \
        -p use_sim_time:=true -p seed:=${SEED:-1} > $WIN_ROOT/$OUT/imu.log 2>&1"
    sleep 5
    in_sim_bg "ros2 run robot_localization ekf_node --ros-args \
        --params-file $WS/scripts/ekf_leo.yaml -r __node:=ekf_filter_node \
        > $WIN_ROOT/$OUT/ekf.log 2>&1"
    sleep 8
  else
    LOG "skid-steer wheel-odometry model (replaces ground-truth TF)"
    in_sim_bg "python3 $WS/scripts/sim_realism_odom.py --ros-args \
        -p use_sim_time:=true -p yaw_scale:=${YAW_SCALE:-0.12} \
        -p linear_scale:=${LINEAR_SCALE:-0.02} -p slip_per_metre:=${SLIP:-0.01} \
        -p seed:=${SEED:-1} > $WIN_ROOT/$OUT/odom.log 2>&1"
    sleep 8
  fi
fi

# ---------- 3. navigation stack ----------
case "$STACK" in
  orig)
    LOG "original stack: slam.launch.py + nav2.launch.py"
    in_sim_bg "ros2 launch leo_rover_gazebo slam.launch.py > $WIN_ROOT/$OUT/slam.log 2>&1"
    in_sim_bg "ros2 launch leo_rover_gazebo nav2.launch.py > $WIN_ROOT/$OUT/nav2.log 2>&1"
    ;;
  bundle|hybrid)
    LOG "overlay stack: sim_navigation.launch.py"
    in_sim_bg "ros2 launch leo_nav2_exploration sim_navigation.launch.py \
        start_slam:=true enable_voxel:=${ENABLE_VOXEL:-true} autostart:=true \
        > $WIN_ROOT/$OUT/overlay.log 2>&1"
    ;;
  *) LOG "FATAL: unknown stack $STACK"; exit 2 ;;
esac

LOG "waiting for Nav2 planner action"
ok=""
for _ in $(seq 1 48); do
  if in_sim 'ros2 action list 2>/dev/null | grep -q compute_path_to_pose'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up"; exit 1; }
sleep 10

CAM_HZ=$(in_sim 'timeout 15 ros2 topic hz /leo1/camera/points 2>&1 | grep -m1 "average rate" || echo none')
LOG "camera/points (with stack up): $CAM_HZ"
FILT_HZ=$(in_sim 'timeout 12 ros2 topic hz /leo1/scan_filtered 2>&1 | grep -m1 "average rate" || echo n/a')
LOG "scan_filtered: $FILT_HZ"

# ---------- 4. recorders ----------
LOG "starting recorders"
in_sim_bg "python3 $WS/scripts/pose_error_recorder.py $WIN_ROOT/$OUT/pose_error.csv 0.5 > $WIN_ROOT/$OUT/pose_rec.log 2>&1"
in_sim_bg "python3 $WS/scripts/map_recorder.py $WIN_ROOT/$OUT/timelapse 10 > $WIN_ROOT/$OUT/timelapse.log 2>&1"
in_sim_bg "python3 $WS/scripts/traj_recorder.py leo1 $WIN_ROOT/$OUT/traj.csv 1.0 > $WIN_ROOT/$OUT/traj.log 2>&1"
# The local costmap as the recovery behaviours see it. `Collision Ahead` says
# the footprint check failed but never what it saw; these frames show it.
if [[ "${USE_COSTMAP_REC:-0}" == "1" ]]; then
  LOG "starting costmap recorder"
  in_sim_bg "python3 $WS/scripts/costmap_recorder.py $WIN_ROOT/$OUT/costmaps ${COSTMAP_PERIOD:-1.0} > $WIN_ROOT/$OUT/costmap_rec.log 2>&1"
fi
if [[ "${USE_ARUCO:-0}" == "1" ]]; then
  LOG "starting ArUco detector"
  in_sim_bg "ros2 launch leo_nav2_exploration aruco.launch.py profile:=sim \
      use_sim_time:=true marker_length:=${ARUCO_LEN:-0.20} \
      registry_file:=$WIN_ROOT/$OUT/aruco_registry.json \
      > $WIN_ROOT/$OUT/aruco.log 2>&1"
fi
sleep 5

# ---------- 5. bootstrap jog ----------
LOG "bootstrap jog"
in_sim 'timeout 10 ros2 topic pub -r 5 /leo1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" >/dev/null 2>&1 || true; ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 5

# ---------- 6. explorer ----------
DONE_PATTERN='Exploration stopped\.'
in_sim_bg "ros2 launch leo_rover_gazebo explore.launch.py > $WIN_ROOT/$OUT/explorer.log 2>&1"
LOG "explorer launched; polling (cap ${CAP_MIN} min)"

# ---------- 7. wait ----------
finished=""; wedged=""
deadline=$(( $(date +%s) + CAP_MIN * 60 ))
last_traj_size=0; stall_checks=0
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if grep -qE "$DONE_PATTERN" "$ROOT/$OUT/explorer.log" 2>/dev/null; then
    finished=1; LOG "explorer reports completion"; break
  fi
  # A wedged simulator (gz-transport "Interrupted system call", /clock frozen)
  # looks exactly like a healthy run from the outside: nodes alive, logs quiet.
  # Watch the trajectory recorder instead -- if it stops growing the clock has
  # stopped, and every further minute of the cap is wasted.
  size=$(stat -c %s "$ROOT/$OUT/traj.csv" 2>/dev/null || echo 0)
  if [[ "$size" == "$last_traj_size" ]]; then
    stall_checks=$((stall_checks + 1))
    if [[ $stall_checks -ge 3 ]]; then
      wedged=1; LOG "FATAL: no trajectory samples for 3 min -- simulator wedged"; break
    fi
  else
    stall_checks=0
  fi
  last_traj_size="$size"
done
[[ -n "$finished" || -n "$wedged" ]] || LOG "cap reached; collecting artefacts anyway"

in_sim 'ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 5

# ---------- 8. save ----------
LOG "saving map"
USE_SIM_TIME=true
[[ -n "$wedged" ]] && USE_SIM_TIME=false   # a frozen /clock stalls the saver
in_sim "ros2 run nav2_map_server map_saver_cli -f $WIN_ROOT/$OUT/map \
    --ros-args -p use_sim_time:=$USE_SIM_TIME -p save_map_timeout:=25.0" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed"

# ---------- 9. teardown ----------
LOG "flushing recorders"
in_sim 'pkill -INT -f "pose_error_recorder[.]py" || true; pkill -INT -f "map_recorder[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true' || true
sleep 15
cleanup_sim
LOG "done -> $OUT"
[[ -n "$finished" ]] || exit 3
