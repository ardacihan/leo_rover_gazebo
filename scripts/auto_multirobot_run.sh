#!/usr/bin/env bash
# Fully automated headless TWO-ROBOT run of the *integrated* stack:
# per-robot SLAM -> real ArUco detection -> tag+grid alignment -> shared map,
# with collaborative frontier exploration on top.
#
#   auto_multirobot_run.sh <mode> <world> <outdir-rel-to-repo> [cap_min]
#
#   mode:   coordinated | independent | single
#   world:  husarion_office | office_world | depot_world
#   outdir: e.g. reports/multirobot_2026-08-23/phase1_husarion_coordinated
#
# How this differs from auto_collab_run.sh, which it replaces for the
# multi-robot work -- each difference removes something that made the old
# numbers unearnable:
#
#   * `gt_odom_tf:=false`. Gazebo's OdometryPublisher reads the true model pose;
#     bridging it onto /tf hands SLAM a perfect prior no physical rover has.
#     scripts/sim_realism_odom.py owns odom->base_link instead, one per rover
#     with a different seed -- and with `zero_origin:=true`, so each rover's
#     odometry starts at (0, 0, 0) the way real wheel odometry does. Without
#     that the node seeds on the true world pose, which puts BOTH SLAM maps in
#     the world frame and makes the leo2->leo1 transform identity by
#     construction: the rovers would secretly share a frame from the first
#     scan, which is precisely what this night is supposed to not assume.
#   * **No map_merge_leo.launch.py.** Its `map -> leo{i}/map` statics are
#     identity, which is only true under ground-truth odometry. The rovers here
#     start in different rooms knowing nothing about each other.
#   * **No `alignment_mode:=fixed`.** That mode publishes the true spawn offset
#     as a static transform, i.e. it hands the merger the answer. This runs
#     `hybrid`: recover from ArUco tags, cross-check by grid matching.
#   * The ground-truth offset is passed only to `compare_to_ground_truth`
#     scoring and to the alignment recorder. No node consumes it for alignment.
#
# The exact launch command lines are echoed into cmdlines.txt so the "no
# cheating" claim can be checked against what actually ran rather than believed.
set -eo pipefail

MODE="$1"; WORLD="$2"; OUT="$3"; CAP_MIN="${4:-25}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

# World-frame clip for the coverage metric. Coverage from a merged grid
# over-counts badly under drift -- phantom cells land outside the building --
# so every condition is measured on the same footprint.
case "$WORLD" in
  office_world)     BOUNDS="-12 12 -8 8" ;;
  depot_world)      BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  husarion_office)  BOUNDS="-4 27 -15 4" ;;
  *)                BOUNDS="" ;;
esac

in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

# Ground truth: leo2's map origin in leo1's map frame, from the same table the
# launch file spawns from (src/leo_rover_gazebo/launch/spawn_poses.py), so the
# error being reported cannot drift from the geometry that was simulated.
# Git-bash on this host has `python` but no `python3`; a CI box may have only
# `python3`. An empty result here would silently score the alignment against
# (0, 0, 0) instead of the true spawn offset, so pick an interpreter that
# works and refuse to run without an answer.
PYBIN=""
for cand in python3 python py; do
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

echo "run: mode=$MODE world=$WORLD cap=${CAP_MIN}min started $(date -Iseconds)" \
  > "$ROOT/$OUT/cmdlines.txt"

# ---------- 0. clean process table ----------
# Stale C++ Nav2 / slam_toolbox binaries from a previous run poison the next
# one; stopping the container is the only reliable way to be rid of them.
LOG "stopping any previous sim container"
docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

# ---------- 1. sim ----------
LOG "starting sim: world=$WORLD robots=$NUM_ROBOTS cameras=on gt_odom_tf=FALSE"
CMD "sim: WORLD=$WORLD GUI=false NUM_ROBOTS=$NUM_ROBOTS ENABLE_CAMERA=true GT_ODOM_TF=false scripts/sim_gpu_wsl.sh"
WORLD="$WORLD" GUI=false NUM_ROBOTS="$NUM_ROBOTS" ENABLE_CAMERA=true \
  GT_ODOM_TF=false "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

LAST_NS="leo${NUM_ROBOTS}"
LOG "waiting for /$LAST_NS/scan"
ok=""
for _ in $(seq 1 48); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/$LAST_NS/scan\$'"; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topics never appeared"; docker logs --tail 60 leo_sim >>"$ROOT/$OUT/run.log" 2>&1; exit 1; }

# ---------- 2. GPU check, inside the first two minutes ----------
# Cameras are on this run, so RGBD rendering is the GPU load. Without
# /usr/lib/wsl/lib on the headless server's LD_LIBRARY_PATH, Ogre silently
# falls back to llvmpipe and everything takes ~6x longer for no reason.
{
  echo "=== nvidia-smi (host) ==="
  nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv 2>&1 || true
  echo "=== Ogre renderer (the check that actually decides) ==="
  in_sim 'for i in $(seq 1 12); do
            if grep -qiE "GL_RENDERER" /root/.ignition/rendering/ogre2.log 2>/dev/null; then
              grep -iE "GL_VERSION|GL_RENDERER|Device Name" /root/.ignition/rendering/ogre2.log | head -4
              exit 0
            fi
            sleep 5
          done
          echo "NO RENDERER LINE after 60s"' 2>&1 || true
} > "$ROOT/$OUT/gpu_check.txt" 2>&1
if grep -qi 'llvmpipe' "$ROOT/$OUT/gpu_check.txt"; then
  LOG "WARNING: Gazebo is on llvmpipe (software GL). Everything will be ~6x slower."
else
  LOG "GPU: $(grep -i 'GL_RENDERER' "$ROOT/$OUT/gpu_check.txt" | head -1 | sed 's/^[0-9:]*//')"
fi

# ---------- 3. realistic odometry: wheel + gyro, fused by an EKF ----------
# This mirrors the real rover exactly, and the mirroring is the point. Feeding
# slam_toolbox RAW wheel odometry is *harsher* than the hardware: a skid-steer's
# wheel yaw carries a ~12% systematic scale error, and in the 2026-08-23 runs
# that cost leo2's SLAM its heading entirely (~114 deg out, map shattered),
# which took the tag alignment down with it. The physical rover does not run
# open-loop wheel yaw -- it runs this EKF, taking forward velocity from the
# wheels and yaw RATE from the gyro. The sim publishes an IMU that nothing was
# consuming.
#
# The IMU's orientation quaternion is deliberately never fused: in Gazebo it is
# derived from ground truth, so fusing it would hand the filter the answer.
# scripts/sim_realism_imu.py marks it invalid and the EKF config takes yaw rate
# only.
LOG "starting realistic odometry (wheel + degraded gyro -> EKF), one per rover"
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  CMD "odom($ns): sim_realism_odom.py (publish_tf=false, zero_origin=true) + sim_realism_imu.py + robot_localization ekf_node"
  # publish_tf:=false -- the EKF owns odom->base_link now.
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args     -p use_sim_time:=true -p input_topic:=/$ns/odom     -p output_topic:=/$ns/odom_wheel_like     -p odom_frame:=$ns/odom -p base_frame:=$ns/base_link     -p publish_tf:=false -p zero_origin:=true     -p seed:=$i > /ros2_ws/$OUT/odom_$ns.log 2>&1"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args     -p use_sim_time:=true -p input_topic:=/$ns/imu/data     -p output_topic:=/$ns/imu/data_real     -p seed:=$i > /ros2_ws/$OUT/imu_$ns.log 2>&1"
  in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args     -r __node:=ekf_filter_node -r __ns:=/$ns     --params-file /ros2_ws/scripts/ekf_$ns.yaml     -r /tf:=/tf -r /tf_static:=/tf_static     > /ros2_ws/$OUT/ekf_$ns.log 2>&1"
done
sleep 8

# The EKF must actually own odom->base_link before SLAM starts, or slam_toolbox
# comes up with no odom prior at all and never recovers.
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

# slam_toolbox publishes to an ABSOLUTE /map; the per-robot remap in
# slam_multi.launch.py is load-bearing. Zero publishers here means both rovers
# clobbered each other on one /map and there are no per-robot maps to merge.
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  n=$(in_sim "ros2 topic info /$ns/map 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+'" || echo 0)
  n=${n//[^0-9]/}; n=${n:-0}
  LOG "  /$ns/map publisher count = $n"
  echo "/$ns/map publishers=$n" >> "$ROOT/$OUT/cmdlines.txt"
  [[ "$n" -ge 1 ]] || { LOG "FATAL: /$ns/map has no publisher -- the slam_multi remap regressed"; exit 1; }
done

# ---------- 5. ArUco detectors ----------
# The hardware-validated detector, one per rover, publishing the MarkerArray
# contract the aligner consumes. marker_length is the plate side: the sim
# textures carry no quiet zone (the world geometry does), so it is 0.20 m --
# not the 0.15 default, which would put every tag 25% short along the view
# ray. Measured in the depot smoke: 0.20 gives a 3.8% along-ray error.
#
# max_range is 4.5, not the 6.0 default. The smoke showed a marker at 5 m
# (25 px across) producing map positions that wandered 8 m as the rover's
# yaw estimate moved, while 2.1 m detections were stable to a few cm. A
# landmark placed badly is worse than one not placed at all, because it is
# persistent and anchors the outlier gate.
LOG "starting ArUco detectors (marker_length=0.20, frame_is_optical=false)"
for i in $(seq 1 "$NUM_ROBOTS"); do
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
  LOG "starting alignment + shared map merger (alignment_mode=hybrid, NOT fixed)"
  CMD "align: ros2 launch multi_robot_shared_mapping shared_align.launch.py alignment_mode:=hybrid enable_tag_alignment:=true enable_map_alignment:=true compare_to_ground_truth:=true ground_truth_x:=$GT_X ground_truth_y:=$GT_Y ground_truth_yaw:=$GT_YAW"
  in_sim_bg "exec ros2 launch multi_robot_shared_mapping shared_align.launch.py \
    use_sim_time:=true alignment_mode:=hybrid \
    enable_tag_alignment:=true enable_map_alignment:=true \
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
# slam_toolbox needs motion before it will publish a map at all, and a rover
# whose first frontier goal lands inside Nav2's goal tolerance "succeeds"
# without moving and can be mistaken for a finished exploration.
LOG "bootstrap jog"
in_sim "for i in \$(seq 1 $NUM_ROBOTS); do (timeout 8 ros2 topic pub -r 5 /leo\$i/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}' >/dev/null 2>&1 &) ; done; sleep 9; for i in \$(seq 1 $NUM_ROBOTS); do ros2 topic pub --once /leo\$i/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; done"
sleep 4

# ---------- 9. monitors ----------
LOG "starting monitors (coverage, trajectories, alignment trace, time-lapse)"
COV_TOPIC="/shared_map"; TRAJ_FRAME="leo1/map"
[[ "$NUM_ROBOTS" -eq 1 ]] && { COV_TOPIC="/leo1/map"; TRAJ_FRAME="leo1/map"; }
ROBOTS=$(seq -s, 1 "$NUM_ROBOTS" | sed 's/\([0-9]\)/leo\1/g')

in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS $COV_TOPIC > /ros2_ws/$OUT/coverage.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ROBOTS /ros2_ws/$OUT/traj.csv 2.0 $TRAJ_FRAME > /ros2_ws/$OUT/traj.log 2>&1"
# Per-rover coverage and trajectory in each rover's OWN map frame, always.
# The two monitors above both depend on the shared frame: /shared_map does not
# publish until alignment locks, and leo2/base_link is not reachable from
# leo1/map until alignment_tf_bridge starts broadcasting. In the Phase 1 run
# that cost the entire coverage curve and every leo2 trajectory sample -- a run
# whose alignment fails must still leave evidence of what each rover did.
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS /$ns/map > /ros2_ws/$OUT/coverage_$ns.log 2>&1"
  in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ns /ros2_ws/$OUT/traj_$ns.csv 2.0 $ns/map > /ros2_ws/$OUT/traj_$ns.log 2>&1"
done
# map_recorder.py subscribes to a hardcoded /map, which does not exist in
# this stack -- it silently recorded nothing all night. This records the
# two per-robot grids, the shared grid, both poses and the alignment as
# raw data, so the merge time-lapse can be re-rendered without re-running.
in_sim_bg "exec python3 /ros2_ws/scripts/merge_timelapse_recorder.py /ros2_ws/$OUT/timelapse 4 > /ros2_ws/$OUT/timelapse.log 2>&1"
if [[ "$NUM_ROBOTS" -eq 2 ]]; then
  in_sim_bg "exec python3 /ros2_ws/scripts/alignment_recorder.py /ros2_ws/$OUT/alignment.csv $GT_X $GT_Y $GT_YAW 5.0 > /ros2_ws/$OUT/alignment.log 2>&1"
fi
# Camera frames across the whole run (a frame grabbed at teardown is whatever
# wall the rover stopped facing). Prefers frames with markers drawn on them.
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/frame_grabber.py /ros2_ws/$OUT/frames_$ns /$ns/camera/image /$ns/aruco/debug_image 25 16 > /ros2_ws/$OUT/frames_$ns.log 2>&1"
done

# ---------- 10. explorers ----------
EXPLORE_MODE="$MODE"; [[ "$MODE" == "single" ]] && EXPLORE_MODE="independent"
LOG "launching explorers (mode=$EXPLORE_MODE, common_frame=leo1/map)"
CMD "explore: ros2 launch leo_rover_exploration collab_explore.launch.py num_robots:=$NUM_ROBOTS coordination_mode:=$EXPLORE_MODE common_frame:=leo1/map"
# Per-rover world extent, so frontier detection stops chasing cells outside
# the building. Same source as the spawn poses, so they cannot drift apart.
L1B="$("$PYBIN" "$ROOT/src/leo_rover_gazebo/launch/spawn_poses.py" "$WORLD" leo1 2>/dev/null || true)"
L2B="$("$PYBIN" "$ROOT/src/leo_rover_gazebo/launch/spawn_poses.py" "$WORLD" leo2 2>/dev/null || true)"
LOG "  frontier bounds: leo1 [$L1B] leo2 [$L2B]"
in_sim_bg "exec ros2 launch leo_rover_exploration collab_explore.launch.py \
  num_robots:=$NUM_ROBOTS coordination_mode:=$EXPLORE_MODE \
  common_frame:=leo1/map leo1_bounds:=$L1B leo2_bounds:=$L2B \
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
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died mid-run"
    docker logs --tail 100 leo_sim >>"$ROOT/$OUT/run.log" 2>&1 || true
    exit 1
  fi
  # Gazebo's headless server has segfaulted inside the WSL D3D12 driver
  # ("D3D12: Removing Device" then SIGSEGV in libd3d12core.so) on long
  # two-camera runs. The container stays up and every ROS node keeps running,
  # so the only symptom is that sim time stops. Without this check the poll
  # loop happily counts down its full 25 minutes against a dead simulator.
  sim_t=$(tail -1 "$ROOT/$OUT/traj_leo1.csv" 2>/dev/null | cut -d, -f1)
  if [[ -n "$sim_t" && "$sim_t" == "$prev_sim_t" ]]; then
    stalls=$((stalls + 1))
    if [[ "$stalls" -ge 2 ]]; then
      LOG "FATAL: sim time frozen at ${sim_t}s for 2 polls - Gazebo is dead."
      docker logs --tail 40 leo_sim 2>&1 | grep -iE "d3d12|segmentation|process has died"         | tail -10 | tee -a "$ROOT/$OUT/run.log" || true
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
  # NOT map_saver_cli: it only ever subscribes TRANSIENT_LOCAL, and
  # shared_map_merger publishes VOLATILE, so the stock saver can never
  # receive /shared_map -- the one artifact the whole pipeline produces.
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
LOG "flushing recorders and stopping the container"
in_sim 'pkill -INT -f "merge_timelapse_recorder[.]py" || true; pkill -INT -f "map_coverage[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true; pkill -INT -f "alignment_recorder[.]py" || true' || true
sleep 12
docker stop leo_sim >/dev/null 2>&1 || true
LOG "done -> $OUT"
[[ -n "$finished" ]] || exit 3
