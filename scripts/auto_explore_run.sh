#!/usr/bin/env bash
# Fully automated headless exploration run.
#
# Usage:
#   auto_explore_run.sh <mode> <world> <outdir-rel-to-repo> [markers_file] [timeout_min]
#
#   mode:         explore_lite | custom | item_search
#   world:        world name or absolute container path to an .sdf
#   outdir:       e.g. reports/final_runs/office_world (created if missing)
#   markers_file: container path to mock markers yaml (item_search only)
#   timeout_min:  wall-clock cap, default 150
#
# Starts the leo_sim container headless, brings up SLAM + Nav2, records
# coverage + time-lapse, runs the requested explorer, waits for completion,
# saves the map, and tears everything down.
set -eo pipefail

MODE="$1"; WORLD="$2"; OUT="$3"; MARKERS="${4:-}"; TIMEOUT_MIN="${5:-150}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "$MODE" || -z "$WORLD" || -z "$OUT" ]]; then
  echo "usage: $0 <mode> <world> <outdir> [markers_file] [timeout_min]" >&2
  exit 2
fi

mkdir -p "$ROOT/$OUT"
LOG() { echo "[auto_explore $(date +%H:%M:%S)] $*"; }

in_sim() { docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

# ---------- 1. sim ----------
LOG "starting sim: world=$WORLD"
# RGBD camera stays on unless the caller sets ENABLE_CAMERA=false.
# It feeds Nav2 costmaps (table legs / low obstacles). SLAM itself is lidar.
CAM="${ENABLE_CAMERA:-true}"
LOG "camera enable_camera=$CAM"
WORLD="$WORLD" GUI=false ENABLE_CAMERA="$CAM" "$ROOT/scripts/sim_gpu_wsl.sh"

LOG "waiting for /leo1/scan"
ok=""
for i in $(seq 1 36); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topic never appeared"; docker logs --tail 40 leo_sim; exit 1; }

# ---------- 2. slam + nav2 ----------
LOG "starting SLAM + Nav2"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam.launch.py > /ros2_ws/$OUT/slam.log 2>&1"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2.launch.py > /ros2_ws/$OUT/nav2.log 2>&1"

LOG "waiting for compute_path_to_pose action"
ok=""
for i in $(seq 1 36); do
  if in_sim 'ros2 action list 2>/dev/null | grep -q compute_path_to_pose'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: Nav2 never came up"; exit 1; }
sleep 10

# ---------- 2b. bootstrap jog ----------
# With a stationary robot slam_toolbox publishes only the initial scan disk,
# whose nearest frontier sits inside Nav2's goal tolerance - explore_lite
# livelocks on an instantly-"reached" goal. Jog ~1 m so the first real map
# exists before any explorer starts (applied to every mode for fairness).
LOG "bootstrap jog"
in_sim 'timeout 10 ros2 topic pub -r 5 /leo1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" >/dev/null 2>&1 || true
ros2 topic pub --once /leo1/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true'
sleep 5

# ---------- 3. monitors ----------
# World-frame clip so the single-robot metric matches the 2-robot footprint.
case "$WORLD" in
  *office*) BOUNDS="-12 12 -8 8" ;;
  *depot*)  BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  *husarion*) BOUNDS="-4 27 -15 4" ;;
  *big*)   BOUNDS="-15 15 -12 12" ;;
  *)        BOUNDS="" ;;
esac
LOG "starting coverage + recorder monitors"
in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS > /ros2_ws/$OUT/coverage.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/map_recorder.py /ros2_ws/$OUT/exploration 10 > /ros2_ws/$OUT/recorder.log 2>&1"

# ---------- 4. explorer ----------
DONE_PATTERN='Exploration finished\.'
case "$MODE" in
  explore_lite)
    DONE_PATTERN='Exploration stopped\.'
    in_sim_bg "exec ros2 launch leo_rover_gazebo explore.launch.py > /ros2_ws/$OUT/explorer.log 2>&1"
    ;;
  custom)
    in_sim_bg "exec ros2 launch leo_rover_exploration frontier_explorer.launch.py \
      params_file:=/ros2_ws/scripts/comparison_params_custom.yaml \
      map_save_path:=/ros2_ws/$OUT/map > /ros2_ws/$OUT/explorer.log 2>&1"
    ;;
  item_search)
    [[ -n "$MARKERS" ]] || { LOG "FATAL: item_search needs markers_file"; exit 2; }
    in_sim_bg "exec ros2 launch leo_rover_exploration item_search.launch.py \
      markers_file:=$MARKERS \
      map_save_path:=/ros2_ws/$OUT/map > /ros2_ws/$OUT/explorer.log 2>&1"
    in_sim_bg "exec ros2 topic echo /frontier_explorer/status std_msgs/msg/String > /ros2_ws/$OUT/status.log 2>&1"
    ;;
  *) LOG "FATAL: unknown mode $MODE"; exit 2 ;;
esac
LOG "explorer ($MODE) launched; polling for completion (cap ${TIMEOUT_MIN} min)"

# ---------- 5. wait ----------
finished=""
deadline=$(( $(date +%s) + TIMEOUT_MIN * 60 ))
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 60
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died mid-run"; exit 1
  fi
  if grep -qE "$DONE_PATTERN" "$ROOT/$OUT/explorer.log" 2>/dev/null; then
    finished=1; LOG "explorer reports completion"; break
  fi
done
[[ -n "$finished" ]] || LOG "WARNING: timeout hit before completion; collecting artifacts anyway"

sleep 15

# ---------- 6. save map (explore_lite has no built-in saver) ----------
if [[ "$MODE" == "explore_lite" ]]; then
  LOG "saving map via map_saver_cli"
  in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/map --ros-args -p use_sim_time:=true" \
    > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver_cli failed (see map_saver.log)"
fi

# ---------- 7. teardown ----------
LOG "flushing recorder (SIGINT) and stopping container"
in_sim 'pkill -INT -f "map_recorder[.]py" || true; pkill -INT -f "map_coverage[.]py" || true' || true
sleep 20
docker stop leo_sim >/dev/null
LOG "done. artifacts in $OUT"
[[ -n "$finished" ]] || exit 3
