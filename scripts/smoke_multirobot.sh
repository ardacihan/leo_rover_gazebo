#!/usr/bin/env bash
# Phase 0 smoke: bring the integrated two-rover stack up on a world, assert the
# things whose absence would waste a full run, tear down. No exploration.
#
#   smoke_multirobot.sh <world> <outdir-rel-to-repo> [cap_min]
#
# What it asserts, and why each one is worth its own check:
#   1. /leo{i}/map has >= 1 publisher -- slam_toolbox publishes to an ABSOLUTE
#      /map and the per-robot remap in slam_multi.launch.py is load-bearing;
#      when it regresses both rovers clobber one /map and there is nothing to
#      merge, with no error anywhere.
#   2. /leo{i}/tag_detections exists AND is a visualization_msgs/MarkerArray --
#      the whole alignment chain is that one contract. The other ArUco node in
#      the tree publishes a String and would satisfy a topic-name check.
#   3. Both rovers actually detect markers -- confirms marker geometry, the
#      0.20 m marker_length, frame_is_optical, and the camera all at once.
#   4. compute_path_to_pose is up for both rovers.
#   5. Gazebo is on the GPU, not llvmpipe.
#   6. odom -> base_link comes from the realism model, and Gazebo's
#      ground-truth TF is NOT on /tf.
set -eo pipefail

WORLD="${1:-depot_world}"; OUT="${2:-reports/multirobot_2026-08-23/smoke}"; CAP_MIN="${3:-12}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/$OUT"
LOG() { echo "[smoke $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/smoke.log"; }
PASS=0; FAIL=0
check() {  # check <name> <expect-nonempty-cmd-output>
  local name="$1"; shift
  local out; out="$("$@" 2>&1 || true)"
  if [[ -n "$out" && "$out" != *"FAILED"* ]]; then
    LOG "  PASS  $name -> $out"; PASS=$((PASS+1))
  else
    LOG "  FAIL  $name -> ${out:-<empty>}"; FAIL=$((FAIL+1))
  fi
}

in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

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
LOG "smoke on $WORLD; ground-truth offset $GT_X $GT_Y $GT_YAW"

docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

LOG "starting sim (2 rovers, cameras on, gt_odom_tf=false)"
WORLD="$WORLD" GUI=false NUM_ROBOTS=2 ENABLE_CAMERA=true GT_ODOM_TF=false \
  "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/smoke.log" 2>&1

ok=""
for _ in $(seq 1 48); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo2/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: /leo2/scan never appeared"; docker logs --tail 60 leo_sim | tee -a "$ROOT/$OUT/smoke.log"; exit 1; }
LOG "sim up"

LOG "GPU check"
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv > "$ROOT/$OUT/nvidia_smi.txt" 2>&1 || true
in_sim 'for i in $(seq 1 12); do
          if grep -qiE "GL_RENDERER" /root/.ignition/rendering/ogre2.log 2>/dev/null; then
            grep -iE "GL_VERSION|GL_RENDERER|Device Name" /root/.ignition/rendering/ogre2.log | head -4
            exit 0
          fi
          sleep 5
        done
        echo "NO RENDERER LINE after 60s"' > "$ROOT/$OUT/renderer.txt" 2>&1 || true
LOG "  renderer: $(tr -d '
' < "$ROOT/$OUT/renderer.txt" | head -3 | tr '
' ' ')"

for i in 1 2; do
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
    -p use_sim_time:=true -p input_topic:=/leo$i/odom -p output_topic:=/leo$i/odom_wheel_like \
    -p odom_frame:=leo$i/odom -p base_frame:=leo$i/base_link -p seed:=$i \
    > /ros2_ws/$OUT/odom_leo$i.log 2>&1"
done
sleep 5

LOG "starting SLAM x2"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=2 > /ros2_ws/$OUT/slam.log 2>&1"
sleep 15

LOG "starting ArUco detectors x2"
for i in 1 2; do
  in_sim_bg "exec ros2 launch leo_nav2_exploration aruco.launch.py \
    profile:=sim use_sim_time:=true robot_ns:=leo$i marker_length:=0.20 \
    max_range:=6.0 min_hits:=3 allowed_ids:=0,1,2,3,4,5,6,7,8,9 \
    detection_topic:=/leo$i/tag_detections markers_topic:=/leo$i/aruco_markers \
    publish_debug_image:=true debug_image_topic:=/{ns}/aruco/debug_image \
    samples_file:=/ros2_ws/$OUT/aruco_samples_leo$i.csv \
    > /ros2_ws/$OUT/aruco_leo$i.log 2>&1"
done

LOG "starting alignment + merger"
in_sim_bg "exec ros2 launch multi_robot_shared_mapping shared_align.launch.py \
  use_sim_time:=true alignment_mode:=hybrid enable_tag_alignment:=true \
  enable_map_alignment:=true enable_alignment_tf:=true \
  compare_to_ground_truth:=true ground_truth_x:=$GT_X ground_truth_y:=$GT_Y \
  ground_truth_yaw:=$GT_YAW > /ros2_ws/$OUT/align.log 2>&1"

LOG "starting Nav2 x2"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=2 > /ros2_ws/$OUT/nav2.log 2>&1"
sleep 20

# Spin both rovers in place: markers are on walls, and a rover that never
# turns may start facing empty floor and detect nothing, which would look
# exactly like a broken detector.
LOG "spinning both rovers to sweep for markers"
in_sim 'for i in 1 2; do (timeout 42 ros2 topic pub -r 5 /leo$i/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.30}}" >/dev/null 2>&1 &); done; sleep 45; for i in 1 2; do ros2 topic pub --once /leo$i/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true; done'
sleep 5

LOG "=== assertions ==="
for i in 1 2; do
  check "/leo$i/map has a publisher" \
    bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 topic info /leo$i/map' | grep -E 'Publisher count: [1-9]'"
  check "/leo$i/tag_detections is a MarkerArray" \
    bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 topic info /leo$i/tag_detections' | grep 'visualization_msgs/msg/MarkerArray'"
  check "leo$i confirmed >=1 ArUco marker" \
    bash -c "grep -c CONFIRMED '$ROOT/$OUT/aruco_leo$i.log' | grep -vE '^0$'"
done
check "two compute_path_to_pose action servers" \
  bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 action list' | grep -c compute_path_to_pose | grep -vE '^[01]$'"
check "shared_map_merger is running" \
  bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 node list' | grep shared_map_merger"
check "alignment_tf_bridge is running" \
  bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 node list' | grep alignment_tf_bridge"
check "odom->base_link exists for leo1" \
  bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; timeout 12 ros2 run tf2_ros tf2_echo leo1/odom leo1/base_link' 2>/dev/null | grep -m1 Translation"

check "Gazebo renders on the GPU, not llvmpipe" \
  bash -c "grep -i GL_RENDERER '$ROOT/$OUT/renderer.txt' | grep -vi llvmpipe"

# The cheat check: Gazebo's ground-truth pose must be diverted off /tf.
check "ground-truth TF diverted off /tf" \
  bash -c "docker exec leo_sim bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 topic list' | grep leo1/tf_ground_truth"

LOG "detector summary:"
for i in 1 2; do
  LOG "  leo$i: $(grep -c CONFIRMED "$ROOT/$OUT/aruco_leo$i.log" 2>/dev/null || echo 0) confirmed; $(grep -oE 'frames=[0-9]+' "$ROOT/$OUT/aruco_leo$i.log" 2>/dev/null | tail -1 || echo 'frames=?')"
  grep -E "CONFIRMED|aruco_detector:" "$ROOT/$OUT/aruco_leo$i.log" 2>/dev/null | head -6 | tee -a "$ROOT/$OUT/smoke.log" || true
done
LOG "alignment log tail:"; tail -6 "$ROOT/$OUT/align.log" 2>/dev/null | tee -a "$ROOT/$OUT/smoke.log" || true

LOG "=== smoke: $PASS passed, $FAIL failed ==="
docker stop leo_sim >/dev/null 2>&1 || true
[[ "$FAIL" -eq 0 ]] || exit 1
