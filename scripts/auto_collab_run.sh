#!/usr/bin/env bash
# Fully automated headless TWO-ROBOT collaborative exploration run.
#
# Usage:
#   auto_collab_run.sh <mode> <world> <outdir-rel-to-repo> [timeout_min]
#
#   mode:    coordinated | independent   (frontier allocation policy)
#   world:   world name or absolute container path to an .sdf
#   outdir:  e.g. reports/collab/office_world_coordinated (created if missing)
#   timeout_min: wall-clock cap, default 60
#
# Brings up the sim with 2 rovers, per-robot SLAM, multirobot_map_merge,
# per-robot Nav2, records merged-map coverage + per-robot trajectories +
# time-lapse, runs both frontier explorers under the chosen coordination mode,
# waits for both to finish, saves the merged map, and tears everything down.
set -eo pipefail

MODE="$1"; WORLD="$2"; OUT="$3"; TIMEOUT_MIN="${4:-60}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$MODE" || -z "$WORLD" || -z "$OUT" ]]; then
  echo "usage: $0 <coordinated|independent> <world> <outdir> [timeout_min]" >&2
  exit 2
fi
if [[ "$MODE" != "coordinated" && "$MODE" != "independent" ]]; then
  echo "FATAL: mode must be coordinated|independent" >&2; exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[auto_collab $(date +%H:%M:%S)] $*"; }

# World-frame clip for the coverage metric (excludes drift phantom outside the
# real world so all conditions are measured on the same footprint).
case "$WORLD" in
  *office*) BOUNDS="-12 12 -8 8" ;;
  *depot*)  BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  *husarion*) BOUNDS="-4 27 -15 4" ;;
  *big*)   BOUNDS="-15 15 -12 12" ;;
  *)        BOUNDS="" ;;
esac
in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

# ---------- 1. sim (2 robots) ----------
LOG "starting sim: world=$WORLD (2 rovers)"
CAM=false; [[ "$WORLD" == *husarion* ]] && CAM=true; WORLD="$WORLD" GUI=false NUM_ROBOTS=2 ENABLE_CAMERA="$CAM" "$ROOT/scripts/sim_gpu_wsl.sh"

LOG "waiting for /leo1/scan and /leo2/scan"
ok=""
for i in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo2/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topics never appeared"; docker logs --tail 40 leo_sim; exit 1; }

# ---------- 2. SLAM x2 + map_merge ----------
LOG "starting per-robot SLAM + map_merge"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=2 > /ros2_ws/$OUT/slam.log 2>&1"
in_sim_bg "exec ros2 launch leo_rover_gazebo map_merge_leo.launch.py > /ros2_ws/$OUT/merge.log 2>&1"
sleep 8

# ---------- 3. Nav2 x2 ----------
LOG "starting per-robot Nav2"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=2 > /ros2_ws/$OUT/nav2.log 2>&1"

LOG "waiting for both compute_path_to_pose action servers"
ok=""
for i in $(seq 1 40); do
  n=$(in_sim 'ros2 action list 2>/dev/null | grep -c compute_path_to_pose || true')
  n=${n:-0}
  if [[ "${n//[^0-9]/}" -ge 2 ]] 2>/dev/null; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up for both robots"; exit 1; }
sleep 10

# ---------- 4. bootstrap jog (both) ----------
LOG "bootstrap jog (both rovers)"
in_sim 'for ns in leo1 leo2; do (timeout 8 ros2 topic pub -r 5 /$ns/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" >/dev/null 2>&1 &) ; done; sleep 9; for ns in leo1 leo2; do ros2 topic pub --once /$ns/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true; done'
sleep 4

# ---------- 5. monitors ----------
LOG "starting coverage + trajectory + time-lapse monitors"
in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS > /ros2_ws/$OUT/coverage.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py leo1,leo2 /ros2_ws/$OUT/traj.csv 2.0 > /ros2_ws/$OUT/traj.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/map_recorder.py /ros2_ws/$OUT/exploration 10 > /ros2_ws/$OUT/recorder.log 2>&1"

# ---------- 6. explorers ----------
LOG "launching collaborative explorers (mode=$MODE)"
in_sim_bg "exec ros2 launch leo_rover_exploration collab_explore.launch.py num_robots:=2 coordination_mode:=$MODE > /ros2_ws/$OUT/explorer.log 2>&1"

# ---------- 7. wait for both to finish ----------
LOG "polling for completion (cap ${TIMEOUT_MIN} min)"
finished=""
deadline=$(( $(date +%s) + TIMEOUT_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died mid-run"; exit 1
  fi
  done_n=$(grep -c 'Exploration finished\.' "$ROOT/$OUT/explorer.log" 2>/dev/null || true)
  done_n=${done_n:-0}
  cov=$(grep -oE 'known=[0-9.]+m2' "$ROOT/$OUT/coverage.log" 2>/dev/null | tail -1 || true)
  LOG "  progress: finished=$done_n/2  coverage=${cov:-?}"
  if [[ "${done_n//[^0-9]/}" -ge 2 ]] 2>/dev/null; then finished=1; LOG "both explorers finished"; break; fi
done
[[ -n "$finished" ]] || LOG "WARNING: timeout before both finished; collecting anyway"
sleep 10

# ---------- 8. save merged + per-robot maps ----------
LOG "saving merged map"
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/merged_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed (see map_saver.log)"
# Per-robot maps are the raw input for offline fusion (scripts/map_fusion.py);
# without them a merged map can never be re-fused with better registration.
for ns in leo1 leo2; do
  LOG "saving per-robot map: $ns"
  in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/${ns}_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map" \
    >> "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli ($ns) failed"
done

# ---------- 9. teardown ----------
LOG "flushing recorders and stopping container"
in_sim 'pkill -INT -f "map_recorder[.]py" || true; pkill -INT -f "map_coverage[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true' || true
sleep 15
docker stop leo_sim >/dev/null
LOG "done. artifacts in $OUT"
[[ -n "$finished" ]] || exit 3
