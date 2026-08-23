#!/usr/bin/env bash
# Fully automated headless ITEM-SEARCH run (1 or 2 robots, cameras ON).
#
# Usage:
#   auto_item_run.sh <n_robots> <mode> <world> <markers_yaml> <outdir> [timeout_min]
#
#   n_robots:     1 | 2
#   mode:         coordinated | independent   (with 1 robot, mode is moot)
#   world:        world name (office_world, depot_world, ...)
#   markers_yaml: container path to the mock-marker ground truth
#   outdir:       e.g. reports/item_search_collab/office_2coord_seedA
#   timeout_min:  wall-clock cap, default 20
#
# Same bring-up as auto_collab_run.sh, plus: cameras enabled, per-robot mock
# ArUco detectors, SWEEPING/VERIFY states (coverage target 0.9), and the
# /item_claims recorder. Saves merged + per-robot maps at the end.
set -eo pipefail

N="$1"; MODE="$2"; WORLD="$3"; MARKERS="$4"; OUT="$5"; TIMEOUT_MIN="${6:-20}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$N" || -z "$MODE" || -z "$WORLD" || -z "$MARKERS" || -z "$OUT" ]]; then
  echo "usage: $0 <1|2> <coordinated|independent> <world> <markers_yaml> <outdir> [timeout_min]" >&2
  exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[auto_item $(date +%H:%M:%S)] $*"; }

case "$WORLD" in
  *office*) BOUNDS="-12 12 -8 8" ;;
  *depot*)  BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  *)        BOUNDS="" ;;
esac
ROBOTS="leo1"; [[ "$N" == "2" ]] && ROBOTS="leo1,leo2"

in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

# ---------- 1. sim (cameras ON: item search needs the RGBD frustum) ----------
LOG "starting sim: world=$WORLD ($N rover(s), cameras on)"
WORLD="$WORLD" GUI=false NUM_ROBOTS="$N" ENABLE_CAMERA=true "$ROOT/scripts/sim_gpu_wsl.sh"

LOG "waiting for scan topic(s)"
LAST="leo$N"
ok=""
for i in $(seq 1 40); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/$LAST/scan$'"; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topics never appeared"; docker logs --tail 40 leo_sim; exit 1; }

# ---------- 2. SLAM + merge (merge also provides map->leo{i}/map TFs) --------
LOG "starting per-robot SLAM + map merge/TFs"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=$N > /ros2_ws/$OUT/slam.log 2>&1"
in_sim_bg "exec ros2 launch leo_rover_gazebo map_merge_leo.launch.py > /ros2_ws/$OUT/merge.log 2>&1"
sleep 8

# ---------- 3. Nav2 ----------
LOG "starting per-robot Nav2"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=$N > /ros2_ws/$OUT/nav2.log 2>&1"

LOG "waiting for $N compute_path_to_pose action server(s)"
ok=""
for i in $(seq 1 40); do
  n=$(in_sim 'ros2 action list 2>/dev/null | grep -c compute_path_to_pose || true')
  n=${n:-0}
  if [[ "${n//[^0-9]/}" -ge "$N" ]] 2>/dev/null; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up"; exit 1; }
sleep 10

# ---------- 4. bootstrap jog ----------
LOG "bootstrap jog"
in_sim "for ns in ${ROBOTS//,/ }; do (timeout 8 ros2 topic pub -r 5 /\$ns/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}' >/dev/null 2>&1 &) ; done; sleep 9; for ns in ${ROBOTS//,/ }; do ros2 topic pub --once /\$ns/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; done"
sleep 4

# ---------- 5. monitors ----------
LOG "starting coverage + trajectory + item recorders"
in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS > /ros2_ws/$OUT/coverage.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ROBOTS /ros2_ws/$OUT/traj.csv 2.0 > /ros2_ws/$OUT/traj.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/item_recorder.py /ros2_ws/$OUT/items.jsonl $ROBOTS > /ros2_ws/$OUT/items_recorder.log 2>&1"

# ---------- 6. explorers + detectors ----------
LOG "launching item-search explorers (n=$N mode=$MODE markers=$MARKERS)"
in_sim_bg "exec ros2 launch leo_rover_exploration collab_explore.launch.py num_robots:=$N coordination_mode:=$MODE item_search:=true markers_file:=$MARKERS > /ros2_ws/$OUT/explorer.log 2>&1"

# ---------- 7. wait ----------
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
  items=$(grep -oE '"items_confirmed": [0-9]+' "$ROOT/$OUT/items.jsonl" 2>/dev/null | tail -1 || true)
  LOG "  progress: finished=$done_n/$N  ${items:-no-items-yet}"
  if [[ "${done_n//[^0-9]/}" -ge "$N" ]] 2>/dev/null; then finished=1; LOG "all explorers finished"; break; fi
done
[[ -n "$finished" ]] || LOG "WARNING: timeout before explorers finished; collecting anyway"
sleep 10

# ---------- 8. save maps ----------
LOG "saving maps"
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/merged_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli (merged) failed"
for ns in ${ROBOTS//,/ }; do
  in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/${ns}_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map" \
    >> "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli ($ns) failed"
done

# ---------- 9. teardown ----------
LOG "stopping container"
in_sim 'pkill -INT -f "item_recorder[.]py" || true; pkill -INT -f "map_coverage[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true' || true
sleep 10
docker stop leo_sim >/dev/null
LOG "done. artifacts in $OUT"
[[ -n "$finished" ]] || exit 3
