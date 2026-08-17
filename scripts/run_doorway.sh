#!/usr/bin/env bash
# Run the bundle's eight-crossing doorway regression against this repo's
# simulator, using the corrected launch file.
#
# The fixture is a two-room wall with a 0.78 m clear doorway; the rover is a
# 0.42 m square with 0.01 m footprint padding, so there is 0.17 m of slack per
# side. Acceptance (from config/sim/doorway_goals.yaml) is 8/8 crossings, zero
# contacts, at most one planner failure, at most two recovery cycles per goal.
#
# Usage:
#   run_doorway.sh <outdir-rel-to-repo> [cap_min] [--lidar-only]
set -eo pipefail

OUT="$1"; CAP_MIN="${2:-25}"; MODE="${3:-}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -n "$OUT" ]] || { echo "usage: $0 <outdir> [cap_min] [--lidar-only]" >&2; exit 2; }

VOXEL=true
[[ "$MODE" == "--lidar-only" ]] && VOXEL=false

mkdir -p "$ROOT/$OUT"
LOG() { echo "[doorway $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }

HOST_ROOT="$ROOT"
if command -v cygpath >/dev/null 2>&1; then
  export MSYS_NO_PATHCONV=1
  HOST_ROOT="$(cygpath -w "$ROOT")"
fi

docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

LOG "starting doorway fixture (voxel=$VOXEL, cap ${CAP_MIN}min)"
docker run -d --name leo_sim \
  --gpus all --device=/dev/dxg \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics \
  -e DISPLAY= \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e GALLIUM_DRIVER=d3d12 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-dir \
  -e ROS2_WS=/ros2_ws \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v "${HOST_ROOT}:/ros2_ws" \
  -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so:ro" \
  -v "${HOST_ROOT}/docker/patched/RenderSystem_GL3Plus.so.2.2.5:/usr/lib/x86_64-linux-gnu/OGRE-Next/RenderSystem_GL3Plus.so.2.2.5:ro" \
  --entrypoint bash \
  leo_rover_humble:bundle \
  -lc "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:\${LD_LIBRARY_PATH} && \
       source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
       export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/leo_rover_description/share:/ros2_ws/src/husarion_gz_worlds/models && \
       mkdir -p /tmp/runtime-dir && chmod 700 /tmp/runtime-dir && \
       mkdir -p /usr/local/share/leo_rover_gazebo && \
       touch /usr/local/share/leo_rover_gazebo/ogre_wsl_gpu_patched && \
       ros2 launch leo_nav2_exploration sim_doorway_regression_leo.launch.py \
         run_regression:=true enable_voxel:=$VOXEL \
         result_file:=/ros2_ws/$OUT/doorway_result.json" >/dev/null

in_sim() { docker exec leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

LOG "waiting for /leo1/scan"
ok=""
for _ in $(seq 1 40); do
  if in_sim 'ros2 topic list 2>/dev/null | grep -q "^/leo1/scan$"'; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: sim never came up"; docker logs --tail 40 leo_sim; exit 1; }

# Recorders, so a failed crossing leaves evidence rather than just a verdict.
sleep 25
in_sim "nohup python3 /ros2_ws/scripts/pose_error_recorder.py /ros2_ws/$OUT/pose_error.csv 0.5 > /ros2_ws/$OUT/pose_rec.log 2>&1 &" || true
in_sim "nohup python3 /ros2_ws/scripts/map_recorder.py /ros2_ws/$OUT/timelapse 10 > /ros2_ws/$OUT/timelapse.log 2>&1 &" || true

LOG "regression running"
deadline=$(( $(date +%s) + CAP_MIN * 60 ))
done_flag=""
while [[ $(date +%s) -lt $deadline ]]; do
  sleep 30
  if [[ -f "$ROOT/$OUT/doorway_result.json" ]]; then done_flag=1; LOG "result file written"; break; fi
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then LOG "container exited"; break; fi
done
[[ -n "$done_flag" ]] || LOG "cap reached without a result file"

docker logs leo_sim > "$ROOT/$OUT/launch.log" 2>&1 || true
in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/map --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0" \
  > "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "map_saver failed"
in_sim 'pkill -INT -f "pose_error_recorder[.]py" || true; pkill -INT -f "map_recorder[.]py" || true' || true
sleep 20
docker stop leo_sim >/dev/null 2>&1
LOG "done -> $OUT"
