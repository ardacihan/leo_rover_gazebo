#!/usr/bin/env bash
# Save the current maps and evidence, render the run media, then stop Gazebo.
#
#   scripts/stop_two_robots_ubuntu.sh <output-directory>
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-}"
if [[ -z "$OUT" || ! -d "$ROOT/$OUT" ]]; then
  echo "usage: $0 <output-directory printed by run_two_robots_ubuntu.sh>" >&2
  exit 2
fi
if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
  echo "FATAL: leo_sim is not running" >&2
  exit 1
fi

in_sim() {
  docker exec leo_sim bash -lc \
    "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"
}

echo "Stopping both rovers safely"
for i in 1 2; do
  in_sim "ros2 topic pub --once /leo$i/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true"
done

echo "Saving leo1, leo2 and shared occupancy grids"
for i in 1 2; do
  in_sim "ros2 run nav2_map_server map_saver_cli \
    -f /ros2_ws/$OUT/leo${i}_map --ros-args \
    -p use_sim_time:=true -p save_map_timeout:=20.0 \
    -r map:=/leo${i}/map" >>"$ROOT/$OUT/map_saver.log" 2>&1 || true
done
in_sim "python3 /ros2_ws/scripts/save_map_volatile.py \
  /shared_map /ros2_ws/$OUT/merged_map 30" \
  >>"$ROOT/$OUT/map_saver.log" 2>&1 || true

in_sim "ros2 node list | sort" >"$ROOT/$OUT/final_nodes.txt" 2>&1 || true
in_sim "ros2 topic list -t | sort" >"$ROOT/$OUT/final_topics.txt" 2>&1 || true
curl --fail --silent "http://127.0.0.1:${DASHBOARD_PORT:-8080}/api/status" \
  >"$ROOT/$OUT/status_final.json" 2>/dev/null || true

# Files are created by root inside the container; return ownership to the
# desktop operator before the container disappears.
docker exec leo_sim chown -R "$(id -u):$(id -g)" "/ros2_ws/$OUT" || true

echo "Stopping simulator container"
docker stop leo_sim >/dev/null

echo "Rendering saved maps, paths, coverage and alignment"
WORLD="$(sed -n 's/^run: mode=[^ ]* world=\([^ ]*\).*/\1/p' \
  "$ROOT/$OUT/cmdlines.txt" 2>/dev/null | head -1)"
WORLD="${WORLD:-office_world}"
TF_ARGS=()
if [[ -s "$ROOT/$OUT/status_final.json" ]]; then
  read -r TF_X TF_Y TF_YAW <<<"$(python3 - "$ROOT/$OUT/status_final.json" <<'PY'
import json, sys
try:
    transform = json.load(open(sys.argv[1]))['alignment']['transform']
    print(transform['x'], transform['y'], transform['yaw'])
except (KeyError, TypeError, OSError, ValueError):
    pass
PY
)"
  if [[ -n "${TF_X:-}" && -n "${TF_Y:-}" && -n "${TF_YAW:-}" ]]; then
    TF_ARGS=(--leo2-tf "$TF_X" "$TF_Y" "$TF_YAW")
  fi
fi
python3 "$ROOT/scripts/render_multirobot_media.py" "$ROOT/$OUT" \
  --world "$WORLD" --title "Two-rover interactive Ubuntu run" \
  "${TF_ARGS[@]}" \
  >>"$ROOT/$OUT/render.log" 2>&1 || true

echo "Building offline recording-analysis dashboard"
python3 "$ROOT/scripts/generate_recording_dashboard.py" "$ROOT/$OUT" \
  >>"$ROOT/$OUT/render.log" 2>&1 || true

echo "Saved run: $ROOT/$OUT"
echo "Dashboard data: $ROOT/$OUT/telemetry.jsonl"
echo "Recording analysis: $ROOT/$OUT/recording_analysis.html"
