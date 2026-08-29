#!/usr/bin/env bash
# VIPER (MPCDF) adaptation of auto_multirobot_run.sh -- same sequence, same
# knobs, but the sim lives in a long-lived apptainer instance instead of the
# leo_sim docker container, and rendering is llvmpipe (software) by design.
#
#   auto_multirobot_run_viper.sh <mode> <world> <outdir-rel-to-ws> [cap_min]
#
# Runs INSIDE an sbatch job on an apu node (see leo_run.sbatch). The workspace
# /ptmp/akalenik/leo_sim/ros2_ws is bound at /ros2_ws, exactly the path the
# colcon symlink-install was built for. Differences from the docker original,
# and nothing else is changed:
#
#   * in_sim()/in_sim_bg() wrap plain `apptainer exec --contain` calls instead
#     of `docker exec leo_sim`. NOT `apptainer instance`: an instance daemonizes
#     out of the Slurm job step and this cluster reaps it within seconds (jobs
#     11006752/11006753 died with "no instance found" while the batch script
#     lived on). Plain execs create no PID namespace, so every container
#     process is an ordinary host process in the job's cgroup: they share the
#     host network (DDS works across execs), they are pkill-able, and they die
#     with the job. --contain hides /dev/dri, which is what makes Mesa fall
#     back to llvmpipe instead of selecting the MI300A's DRM device it cannot
#     open and then segfaulting OGRE (job 11006598 crashed, 11006602 ran at
#     RTF 0.95).
#   * CYCLONEDDS_URI pins DDS to 127.0.0.1 with MaxAutoParticipantIndex=119.
#     Compute nodes are multi-homed and CycloneDDS's default interface pick
#     made discovery fail across execs (jobs 11006678/11006679); loopback is
#     not multicast-capable here, so discovery is unicast index probing, and
#     the default index cap is smaller than this stack's participant count.
#   * The sim is launched with ros2 launch two_robots_gpu.launch.py directly
#     (sim_gpu_wsl.sh is docker-run + WSL-GPU plumbing that has no meaning here).
#   * ENABLE_CAMERA defaults to FALSE: cameras are untested under llvmpipe on
#     Viper and unneeded for marker-free runs. The knob still works if exported.
#   * ROS_DOMAIN_ID and IGN_PARTITION are derived from SLURM_JOB_ID so two
#     jobs that land on the same node cannot cross-talk over DDS or
#     ign-transport.
#   * The llvmpipe warning from the GPU check is informational, not a problem:
#     on Viper llvmpipe IS the plan.
set -eo pipefail

MODE="$1"; WORLD="$2"; OUT="$3"; CAP_MIN="${4:-25}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

SIF="${SIF:-/ptmp/akalenik/leo_sim/leo_bundle.sif}"
DOMAIN=$(( ${SLURM_JOB_ID:-42} % 100 + 1 ))
IGNPART="leo_${SLURM_JOB_ID:-local}"
SIM_PID=""

# Overridable for the marker-free confirmation runs (Phase 1, night 2026-08-25):
#   ALIGN_MODE=markerfree SKIP_ARUCO=1  -> no detectors, no tag aligner; the
#   grid matcher with margin abstention is the only alignment source.
ALIGN_MODE="${ALIGN_MODE:-hybrid}"
TAG_ALIGN="true"
[[ -n "${SKIP_ARUCO:-}" || "$ALIGN_MODE" == "markerfree" ]] && TAG_ALIGN="false"
# Viper: cameras untested under llvmpipe and unneeded -> default false.
ENABLE_CAMERA="${ENABLE_CAMERA:-false}"

if [[ -z "$MODE" || -z "$WORLD" || -z "$OUT" ]]; then
  echo "usage: $0 <coordinated|independent|single> <world> <outdir> [cap_min]" >&2
  exit 2
fi
case "$MODE" in
  coordinated|independent|single) ;;
  *) echo "FATAL: mode must be coordinated|independent|single" >&2; exit 2 ;;
esac

NUM_ROBOTS=2
[[ "$MODE" == "single" ]] && NUM_ROBOTS=1

mkdir -p "$ROOT/$OUT"
LOG() { echo "[multirobot $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }
CMD() { echo "$*" >> "$ROOT/$OUT/cmdlines.txt"; }

# World-frame clip for the coverage metric (same as the docker original).
case "$WORLD" in
  office_world)     BOUNDS="-12 12 -8 8" ;;
  depot_world)      BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  husarion_office)  BOUNDS="-4 27 -15 4" ;;
  *)                BOUNDS="" ;;
esac

# Every exec re-establishes the environment. Loopback-pinned DDS: validated on
# the login node across independent execs, including a high participant index.
CDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address='127.0.0.1'/></Interfaces></General><Discovery><MaxAutoParticipantIndex>119</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
ENVSETUP="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash \
 && export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models:/ros2_ws/src/leo_rover_gazebo/models \
 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=$DOMAIN IGN_PARTITION=$IGNPART \
 && export CYCLONEDDS_URI=\"$CDDS_URI\" \
 && unset DISPLAY && export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe EGL_PLATFORM=surfaceless \
 && export XDG_RUNTIME_DIR=/tmp/runtime-dir && mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir"

in_sim()    { apptainer exec --contain --bind "$ROOT:/ros2_ws" "$SIF" bash -c "$ENVSETUP && $1"; }
in_sim_bg() { apptainer exec --contain --bind "$ROOT:/ros2_ws" "$SIF" bash -c "$ENVSETUP && $1" </dev/null >/dev/null 2>&1 & }

sim_alive() { [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; }

# Ground truth: same contract as the original -- scoring only, never consumed
# for alignment.
PYBIN=""
for cand in python3 python; do
  if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import sys" >/dev/null 2>&1; then
    PYBIN="$cand"; break
  fi
done
[[ -n "$PYBIN" ]] || { echo "FATAL: no working python interpreter on PATH" >&2; exit 1; }
read -r GT_X GT_Y GT_YAW <<<"$("$PYBIN" "$ROOT/src/leo_rover_gazebo/launch/spawn_poses.py" "$WORLD")"
if [[ -z "$GT_X" || -z "$GT_Y" || -z "$GT_YAW" ]]; then
  echo "FATAL: could not resolve the ground-truth spawn offset for $WORLD" >&2
  exit 1
fi
LOG "ground-truth leo2->leo1 offset: x=$GT_X y=$GT_Y yaw=$GT_YAW rad (scoring only)"
LOG "viper: node=$(hostname) job=${SLURM_JOB_ID:-none} instance=$INST domain=$DOMAIN"

echo "run: mode=$MODE world=$WORLD cap=${CAP_MIN}min started $(date -Iseconds) viper_job=${SLURM_JOB_ID:-none}" \
  > "$ROOT/$OUT/cmdlines.txt"

# ---------- 0. clean process table ----------
# Unlike docker, there is nothing to stop: every process of a previous job died
# with that job's cgroup, and a generic pkill here could hit OUR OTHER Slurm
# job if two runs share a node. Deliberately a no-op.

# ---------- 1. sim ----------
LOG "starting sim: world=$WORLD robots=$NUM_ROBOTS cameras=$ENABLE_CAMERA gt_odom_tf=FALSE (apptainer --contain, llvmpipe)"
CMD "sim: apptainer exec --contain --bind $ROOT:/ros2_ws $SIF ros2 launch leo_rover_gazebo two_robots_gpu.launch.py world:=$WORLD gui:=false num_robots:=$NUM_ROBOTS enable_camera:=$ENABLE_CAMERA gt_odom_tf:=false"
in_sim_bg "exec ros2 launch leo_rover_gazebo two_robots_gpu.launch.py \
  world:=$WORLD gui:=false num_robots:=$NUM_ROBOTS \
  enable_camera:=$ENABLE_CAMERA gt_odom_tf:=false \
  > /ros2_ws/$OUT/sim_launch.log 2>&1"
SIM_PID=$!
LOG "sim launch exec pid=$SIM_PID"

LAST_NS="leo${NUM_ROBOTS}"
LOG "waiting for /$LAST_NS/scan"
ok=""
for _ in $(seq 1 48); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/$LAST_NS/scan\$'"; then ok=1; break; fi
  sleep 5
done
if [[ -z "$ok" ]]; then
  LOG "FATAL: scan topics never appeared -- dumping discovery diagnostics"
  {
    echo "--- ros2 topic list (daemon) ---";    in_sim "ros2 topic list" || true
    echo "--- ros2 topic list --no-daemon ---"; in_sim "ros2 topic list --no-daemon --spin-time 5" || true
    echo "--- ros2 daemon status ---";          in_sim "ros2 daemon status" || true
    echo "--- interfaces ---";                  in_sim "cat /proc/net/dev | head -8" || true
    echo "--- sim_launch tail ---";             tail -60 "$ROOT/$OUT/sim_launch.log"
  } >>"$ROOT/$OUT/run.log" 2>&1
  exit 1
fi

# ---------- 2. renderer check ----------
# llvmpipe is EXPECTED here; this records proof of what actually rendered.
{
  echo "=== rocm-smi (host, informational -- rendering is llvmpipe by design) ==="
  rocm-smi 2>&1 | head -8 || true
  echo "=== Ogre renderer (the check that actually decides) ==="
  in_sim 'for i in $(seq 1 12); do
            if grep -qiE "GL_RENDERER" $HOME/.ignition/rendering/ogre2.log 2>/dev/null; then
              grep -iE "GL_VERSION|GL_RENDERER|Device Name" $HOME/.ignition/rendering/ogre2.log | head -4
              exit 0
            fi
            sleep 5
          done
          echo "NO RENDERER LINE after 60s"' 2>&1 || true
} > "$ROOT/$OUT/gpu_check.txt" 2>&1
if grep -qi 'llvmpipe' "$ROOT/$OUT/gpu_check.txt"; then
  LOG "renderer: llvmpipe (software GL) -- expected on Viper, RTF ~0.95 measured"
else
  LOG "renderer: $(grep -i 'GL_RENDERER' "$ROOT/$OUT/gpu_check.txt" | head -1 | sed 's/^[0-9:]*//')"
fi

# ---------- 3. realistic odometry: wheel + gyro, fused by an EKF ----------
LOG "starting realistic odometry (wheel + degraded gyro -> EKF), one per rover"
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  CMD "odom($ns): sim_realism_odom.py (publish_tf=false, zero_origin=true) + sim_realism_imu.py + robot_localization ekf_node"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args     -p use_sim_time:=true -p input_topic:=/$ns/odom     -p output_topic:=/$ns/odom_wheel_like     -p odom_frame:=$ns/odom -p base_frame:=$ns/base_link     -p publish_tf:=false -p zero_origin:=true     -p seed:=$i > /ros2_ws/$OUT/odom_$ns.log 2>&1"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args     -p use_sim_time:=true -p input_topic:=/$ns/imu/data     -p output_topic:=/$ns/imu/data_real     -p seed:=$i > /ros2_ws/$OUT/imu_$ns.log 2>&1"
  in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args     -r __node:=ekf_filter_node -r __ns:=/$ns     --params-file /ros2_ws/scripts/ekf_$ns.yaml     -r /tf:=/tf -r /tf_static:=/tf_static     > /ros2_ws/$OUT/ekf_$ns.log 2>&1"
done
sleep 8

for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  if in_sim "timeout 12 ros2 run tf2_ros tf2_echo $ns/odom $ns/base_link 2>/dev/null | grep -q Translation"; then
    LOG "  $ns/odom -> $ns/base_link is live (EKF)"
  else
    LOG "  WARNING: no $ns/odom -> $ns/base_link yet; SLAM may start blind"
  fi
done

# ---------- 4. per-robot SLAM ----------
LOG "starting per-robot SLAM"
CMD "slam: ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=$NUM_ROBOTS"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=$NUM_ROBOTS > /ros2_ws/$OUT/slam.log 2>&1"
sleep 12

for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  n=$(in_sim "ros2 topic info /$ns/map 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+'" || echo 0)
  n=${n//[^0-9]/}; n=${n:-0}
  LOG "  /$ns/map publisher count = $n"
  echo "/$ns/map publishers=$n" >> "$ROOT/$OUT/cmdlines.txt"
  [[ "$n" -ge 1 ]] || { LOG "FATAL: /$ns/map has no publisher -- the slam_multi remap regressed"; exit 1; }
done

# ---------- 5. ArUco detectors ----------
if [[ -n "${SKIP_ARUCO:-}" ]]; then
  LOG "SKIP_ARUCO set: no ArUco detectors this run (marker-free confirmation)"
  echo "aruco: SKIPPED (SKIP_ARUCO=1)" >> "$ROOT/$OUT/cmdlines.txt"
fi
LOG "starting ArUco detectors (marker_length=0.20, frame_is_optical=false)"
for i in $(seq 1 "$NUM_ROBOTS"); do
  [[ -n "${SKIP_ARUCO:-}" ]] && break
  ns="leo$i"
  CMD "aruco($ns): ros2 launch leo_nav2_exploration aruco.launch.py profile:=sim robot_ns:=$ns marker_length:=0.20 allowed_ids:=0,1,2,3,4,5,6,7,8,9 detection_topic:=/$ns/tag_detections"
  in_sim_bg "exec ros2 launch leo_nav2_exploration aruco.launch.py \
    profile:=sim use_sim_time:=true robot_ns:=$ns \
    marker_length:=0.20 max_range:=4.5 min_hits:=3 \
    allowed_ids:=0,1,2,3,4,5,6,7,8,9 \
    detection_topic:=/$ns/tag_detections \
    markers_topic:=/$ns/aruco_markers \
    publish_debug_image:=true \
    debug_image_topic:=/$ns/aruco/debug_image \
    registry_file:=/ros2_ws/$OUT/aruco_registry_$ns.json \
    samples_file:=/ros2_ws/$OUT/aruco_samples_$ns.csv \
    > /ros2_ws/$OUT/aruco_$ns.log 2>&1"
done
sleep 8

# ---------- 6. alignment + shared map ----------
if [[ "$NUM_ROBOTS" -eq 2 ]]; then
  LOG "starting alignment + shared map merger (alignment_mode=$ALIGN_MODE, tag_alignment=$TAG_ALIGN, NOT fixed)"
  CMD "align: ros2 launch multi_robot_shared_mapping shared_align.launch.py alignment_mode:=$ALIGN_MODE enable_tag_alignment:=$TAG_ALIGN enable_map_alignment:=true compare_to_ground_truth:=true ground_truth_x:=$GT_X ground_truth_y:=$GT_Y ground_truth_yaw:=$GT_YAW"
  in_sim_bg "exec ros2 launch multi_robot_shared_mapping shared_align.launch.py \
    use_sim_time:=true alignment_mode:=$ALIGN_MODE \
    enable_tag_alignment:=$TAG_ALIGN enable_map_alignment:=true \
    enable_alignment_tf:=true min_tags:=2 \
    compare_to_ground_truth:=true \
    ground_truth_x:=$GT_X ground_truth_y:=$GT_Y ground_truth_yaw:=$GT_YAW \
    > /ros2_ws/$OUT/align.log 2>&1"
  sleep 6
fi

# ---------- 7. Nav2 ----------
LOG "starting per-robot Nav2"
CMD "nav2: ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=$NUM_ROBOTS"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=$NUM_ROBOTS > /ros2_ws/$OUT/nav2.log 2>&1"

LOG "waiting for $NUM_ROBOTS compute_path_to_pose action servers"
ok=""
for _ in $(seq 1 40); do
  n=$(in_sim 'ros2 action list 2>/dev/null | grep -c compute_path_to_pose || true')
  n=${n//[^0-9]/}; n=${n:-0}
  if [[ "$n" -ge "$NUM_ROBOTS" ]] 2>/dev/null; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up for all robots"; exit 1; }
sleep 8

# ---------- 8. bootstrap jog ----------
LOG "bootstrap jog"
in_sim "for i in \$(seq 1 $NUM_ROBOTS); do (timeout 8 ros2 topic pub -r 5 /leo\$i/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}' >/dev/null 2>&1 &) ; done; sleep 9; for i in \$(seq 1 $NUM_ROBOTS); do ros2 topic pub --once /leo\$i/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; done"
sleep 4

# ---------- 9. monitors ----------
LOG "starting monitors (coverage, trajectories, alignment trace, time-lapse)"
COV_TOPIC="/shared_map"; TRAJ_FRAME="leo1/map"
[[ "$NUM_ROBOTS" -eq 1 ]] && { COV_TOPIC="/leo1/map"; TRAJ_FRAME="leo1/map"; }
ROBOTS=$(seq -s, 1 "$NUM_ROBOTS" | sed 's/\([0-9]\)/leo\1/g')

# PIDs of the recorder execs: teardown signals exactly these, because a
# name-based pkill would hit our OTHER job's recorders on a shared node.
RECORDER_PIDS=()
in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS $COV_TOPIC > /ros2_ws/$OUT/coverage.log 2>&1"; RECORDER_PIDS+=($!)
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ROBOTS /ros2_ws/$OUT/traj.csv 2.0 $TRAJ_FRAME > /ros2_ws/$OUT/traj.log 2>&1"; RECORDER_PIDS+=($!)
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS /$ns/map > /ros2_ws/$OUT/coverage_$ns.log 2>&1"; RECORDER_PIDS+=($!)
  in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ns /ros2_ws/$OUT/traj_$ns.csv 2.0 $ns/map > /ros2_ws/$OUT/traj_$ns.log 2>&1"; RECORDER_PIDS+=($!)
done
in_sim_bg "exec python3 /ros2_ws/scripts/merge_timelapse_recorder.py /ros2_ws/$OUT/timelapse 4 > /ros2_ws/$OUT/timelapse.log 2>&1"; RECORDER_PIDS+=($!)
if [[ "$NUM_ROBOTS" -eq 2 ]]; then
  in_sim_bg "exec python3 /ros2_ws/scripts/alignment_recorder.py /ros2_ws/$OUT/alignment.csv $GT_X $GT_Y $GT_YAW 5.0 > /ros2_ws/$OUT/alignment.log 2>&1"; RECORDER_PIDS+=($!)
fi
for i in $(seq 1 "$NUM_ROBOTS"); do
  [[ "$ENABLE_CAMERA" == "true" ]] || break
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/frame_grabber.py /ros2_ws/$OUT/frames_$ns /$ns/camera/image /$ns/aruco/debug_image 25 16 > /ros2_ws/$OUT/frames_$ns.log 2>&1"
done

# ---------- 10. explorers ----------
EXPLORE_MODE="$MODE"; [[ "$MODE" == "single" ]] && EXPLORE_MODE="independent"
LOG "launching explorers (mode=$EXPLORE_MODE, common_frame=leo1/map)"
SHARED_TOPIC=""
[[ "$EXPLORE_MODE" == "coordinated" && "$NUM_ROBOTS" -eq 2 ]] && SHARED_TOPIC="/shared_map"
CMD "explore: ros2 launch leo_rover_exploration collab_explore.launch.py num_robots:=$NUM_ROBOTS coordination_mode:=$EXPLORE_MODE common_frame:=leo1/map shared_map_topic:=$SHARED_TOPIC"
L1B="$("$PYBIN" "$ROOT/src/leo_rover_gazebo/launch/spawn_poses.py" "$WORLD" leo1 2>/dev/null || true)"
L2B="$("$PYBIN" "$ROOT/src/leo_rover_gazebo/launch/spawn_poses.py" "$WORLD" leo2 2>/dev/null || true)"
LOG "  frontier bounds: leo1 [$L1B] leo2 [$L2B]"
in_sim_bg "exec ros2 launch leo_rover_exploration collab_explore.launch.py \
  num_robots:=$NUM_ROBOTS coordination_mode:=$EXPLORE_MODE \
  common_frame:=leo1/map leo1_bounds:=$L1B leo2_bounds:=$L2B \
  shared_map_topic:=$SHARED_TOPIC \
  > /ros2_ws/$OUT/explorer.log 2>&1"

# ---------- 11. wait ----------
LOG "polling for completion (cap ${CAP_MIN} min)"
finished=""
gz_dead=""
stalls=0
prev_sim_t=""
deadline=$(( $(date +%s) + CAP_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if ! sim_alive; then
    LOG "FATAL: sim launch process (pid $SIM_PID) died mid-run"
    tail -100 "$ROOT/$OUT/sim_launch.log" >>"$ROOT/$OUT/run.log" 2>&1 || true
    exit 1
  fi
  # Same freeze detection as the docker original: a renderer/physics crash can
  # leave every ROS node alive while sim time stops (locally it was the WSL
  # D3D12 driver; here it would be llvmpipe/OGRE). Two frozen polls = dead.
  sim_t=$(tail -1 "$ROOT/$OUT/traj_leo1.csv" 2>/dev/null | cut -d, -f1)
  if [[ -n "$sim_t" && "$sim_t" == "$prev_sim_t" ]]; then
    stalls=$((stalls + 1))
    if [[ "$stalls" -ge 2 ]]; then
      LOG "FATAL: sim time frozen at ${sim_t}s for 2 polls - Gazebo is dead."
      grep -iE "segmentation|process has died|Err\]" "$ROOT/$OUT/sim_launch.log" 2>/dev/null         | tail -10 | tee -a "$ROOT/$OUT/run.log" || true
      gz_dead=1
      break
    fi
  else
    stalls=0
  fi
  prev_sim_t="$sim_t"
  done_n=$(grep -c 'Exploration finished\.' "$ROOT/$OUT/explorer.log" 2>/dev/null || true)
  done_n=${done_n//[^0-9]/}; done_n=${done_n:-0}
  cov=$(grep -oE 'known=[0-9.]+m2' "$ROOT/$OUT/coverage.log" 2>/dev/null | tail -1 || true)
  align=$(tail -1 "$ROOT/$OUT/alignment.log" 2>/dev/null || true)
  LOG "  finished=$done_n/$NUM_ROBOTS coverage=${cov:-?} | ${align:-no alignment yet}"
  if [[ "$done_n" -ge "$NUM_ROBOTS" ]] 2>/dev/null; then finished=1; LOG "all explorers finished"; break; fi
done
if [[ -n "$gz_dead" ]]; then
  LOG "collecting what exists from the dead-simulator run"
elif [[ -z "$finished" ]]; then
  LOG "WARNING: hit the ${CAP_MIN} min cap before finishing; collecting anyway"
fi
sleep 8

# ---------- 12. save maps ----------
if [[ "$NUM_ROBOTS" -eq 2 ]]; then
  LOG "saving merged map (/shared_map, VOLATILE-compatible saver)"
  in_sim "python3 /ros2_ws/scripts/save_map_volatile.py /shared_map /ros2_ws/$OUT/merged_map 30" \
    > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "merged map not saved (see map_saver.log)"
fi
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  LOG "saving per-robot map: $ns"
  in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/${ns}_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map" \
    >> "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli ($ns) failed"
done

# ---------- 13. teardown ----------
# Container processes are host processes (no PID namespace) and apptainer's
# starter forwards signals, so SIGINT to the recorded exec PIDs reaches the
# python recorders and flushes them. PID-scoped on purpose: a name-based pkill
# would kill our OTHER job's recorders when two runs share a node.
LOG "flushing recorders and stopping the sim"
for p in "${RECORDER_PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
sleep 12
[[ -n "$SIM_PID" ]] && kill -INT "$SIM_PID" 2>/dev/null || true
sleep 5
LOG "done -> $OUT"
[[ -n "$finished" ]] || exit 3
