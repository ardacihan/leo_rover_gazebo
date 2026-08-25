#!/bin/bash
# Minimal bring-up for debugging the rehearsal safety chain: sim + realism
# odom + ONE namespaced real_mapping stack (leo1). No teardown -- the
# container stays up for interactive tracing.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=reports/night_2026-08-25/phase4_rehearsal_debug
mkdir -p "$ROOT/$OUT"
in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true
WORLD=office_world GUI=false NUM_ROBOTS=2 ENABLE_CAMERA=false \
  GT_ODOM_TF=false "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

for _ in $(seq 1 36); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/leo1/scan\$'"; then break; fi
  sleep 5
done

in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
  -p use_sim_time:=true -p input_topic:=/leo1/odom \
  -p output_topic:=/leo1/odom_wheel_like \
  -p odom_frame:=leo1/odom -p base_frame:=leo1/base_link \
  -p publish_tf:=false -p zero_origin:=true -p seed:=1 \
  > /ros2_ws/$OUT/odom_leo1.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args \
  -p use_sim_time:=true -p input_topic:=/leo1/imu/data \
  -p output_topic:=/leo1/imu/data_real -p seed:=1 \
  > /ros2_ws/$OUT/imu_leo1.log 2>&1"
in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node -r __ns:=/leo1 \
  --params-file /ros2_ws/scripts/ekf_leo1.yaml \
  -r /tf:=/tf -r /tf_static:=/tf_static \
  > /ros2_ws/$OUT/ekf_leo1.log 2>&1"
in_sim_bg "exec ros2 run tf2_ros static_transform_publisher \
  --frame-id leo1/base_link --child-frame-id leo1/base_footprint \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1"
sleep 10

in_sim_bg "exec ros2 launch leo_nav2_exploration real_mapping.launch.py \
  robot_ns:=leo1 use_sim_time:=true \
  guard_odom_topic:=/leo1/odom_wheel_like \
  > /ros2_ws/$OUT/real_mapping_leo1.log 2>&1"
sleep 40
in_sim "ros2 topic list > /dev/null 2>&1 || true"
echo "DEBUG STACK UP -> $OUT"
