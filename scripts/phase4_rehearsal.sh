#!/usr/bin/env bash
# Phase 4 rehearsal: the REAL launch path, twice, against the simulator.
#
#   bash scripts/run_snapshot.sh scripts/phase4_rehearsal.sh [outdir] [cap_min]
#
# Brings up two_robots_gpu.launch.py (no cameras) + realism odometry/EKF,
# then real_mapping.launch.py robot_ns:=leo1 and robot_ns:=leo2 -- the same
# launch file the physical rovers will run, with only the two documented sim
# knobs (use_sim_time, guard_odom_topic). Passes when:
#
#   1. both real slam stacks publish /leo{i}/map,
#   2. TF resolves leo{i}/map -> leo{i}/base_link through prefixed frames,
#   3. a teleop command entering the TOP of the safety chain
#      (/leo{i}/cmd_vel_nav -> smoother -> guard -> collision monitor
#       -> /leo{i}/cmd_vel) actually moves the rover.
#
# The sim URDF has no base_footprint link, the real rover does; identity
# statics leo{i}/base_link -> leo{i}/base_footprint bridge that gap here and
# only here.
set -eo pipefail

OUT="${1:-reports/night_2026-08-25/phase4_rehearsal}"
CAP_MIN="${2:-10}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/$OUT"
LOG() { echo "[rehearsal $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }
in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

VERDICT="$ROOT/$OUT/rehearsal_summary.txt"
: > "$VERDICT"
note() { echo "$*" | tee -a "$VERDICT"; }

LOG "stopping any previous sim container"
docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

LOG "starting sim (office_world, 2 robots, cameras off)"
WORLD=office_world GUI=false NUM_ROBOTS=2 ENABLE_CAMERA=false \
  GT_ODOM_TF=false "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

ok=""
for _ in $(seq 1 36); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/leo2/scan\$'"; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topics never appeared"; exit 1; }

LOG "starting realism odometry + EKF (same as the measurement runs)"
for i in 1 2; do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args \
    -p use_sim_time:=true -p input_topic:=/$ns/odom \
    -p output_topic:=/$ns/odom_wheel_like \
    -p odom_frame:=$ns/odom -p base_frame:=$ns/base_link \
    -p publish_tf:=false -p zero_origin:=true \
    -p seed:=$i > /ros2_ws/$OUT/odom_$ns.log 2>&1"
  in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args \
    -p use_sim_time:=true -p input_topic:=/$ns/imu/data \
    -p output_topic:=/$ns/imu/data_real \
    -p seed:=$i > /ros2_ws/$OUT/imu_$ns.log 2>&1"
  in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args \
    -r __node:=ekf_filter_node -r __ns:=/$ns \
    --params-file /ros2_ws/scripts/ekf_$ns.yaml \
    -r /tf:=/tf -r /tf_static:=/tf_static \
    > /ros2_ws/$OUT/ekf_$ns.log 2>&1"
  # the real rover has base_footprint; the sim URDF does not
  in_sim_bg "exec ros2 run tf2_ros static_transform_publisher \
    --frame-id $ns/base_link --child-frame-id $ns/base_footprint \
    --ros-args -p use_sim_time:=true > /dev/null 2>&1"
done
sleep 12

LOG "launching the REAL mapping stack under both namespaces"
for i in 1 2; do
  ns="leo$i"
  in_sim_bg "exec ros2 launch leo_nav2_exploration real_mapping.launch.py \
    robot_ns:=$ns use_sim_time:=true \
    guard_odom_topic:=/$ns/odom_wheel_like \
    > /ros2_ws/$OUT/real_mapping_$ns.log 2>&1"
done
sleep 45
# warm the ros2 CLI daemon once, so the checks below do not spend their
# timeouts on discovery (a 6 s echo returned empty on a cold daemon and
# failed an otherwise-healthy stack)
in_sim "ros2 topic list > /dev/null 2>&1 || true"

# --- check 1: per-robot maps from the real slam nodes -----------------------
for i in 1 2; do
  ns="leo$i"
  n=$(in_sim "ros2 topic info /$ns/map 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+'" || echo 0)
  n=${n//[^0-9]/}; n=${n:-0}
  if [[ "$n" -ge 1 ]]; then
    note "CHECK map($ns): PASS (publisher count $n)"
  else
    note "CHECK map($ns): FAIL (no publisher on /$ns/map)"
  fi
done

# --- check 2: prefixed TF chain --------------------------------------------
# slam needs motion before publishing map->odom; jog first (also feeds check 3).
LOG "teleop jog through the top of the safety chain"
for i in 1 2; do
  ns="leo$i"
  before=$(in_sim "timeout 15 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A3 'position:' | grep 'x:' | head -1 | grep -oE '[-0-9.e]+'" || echo "")
  in_sim "(timeout 15 ros2 topic pub -r 5 /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{linear: {x: 0.15}}' >/dev/null 2>&1 || true); ros2 topic pub --once /$ns/cmd_vel_nav geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true"
  after=$(in_sim "timeout 15 ros2 topic echo --once /$ns/odom_wheel_like 2>/dev/null | grep -m1 -A3 'position:' | grep 'x:' | head -1 | grep -oE '[-0-9.e]+'" || echo "")
  echo "jog($ns): x $before -> $after" >> "$ROOT/$OUT/run.log"
  moved=$(python3 - "$before" "$after" <<'PY' 2>/dev/null || echo no
import sys
try:
    b, a = float(sys.argv[1]), float(sys.argv[2])
    print('yes' if abs(a - b) > 0.15 else 'no')
except Exception:
    print('no')
PY
)
  if [[ "$moved" == "yes" ]]; then
    note "CHECK safety-chain motion($ns): PASS (odom x $before -> $after)"
  else
    note "CHECK safety-chain motion($ns): FAIL (odom x '$before' -> '$after' -- guard, smoother or monitor is holding the rover)"
  fi
done
sleep 10

for i in 1 2; do
  ns="leo$i"
  if in_sim "timeout 15 ros2 run tf2_ros tf2_echo $ns/map $ns/base_link 2>/dev/null | grep -q 'Translation'"; then
    note "CHECK tf($ns/map -> $ns/base_link): PASS"
  else
    note "CHECK tf($ns/map -> $ns/base_link): FAIL"
  fi
done

# --- collect ----------------------------------------------------------------
for i in 1 2; do
  ns="leo$i"
  in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/${ns}_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map" \
    >> "$ROOT/$OUT/map_saver.log" 2>&1 || note "map save ($ns) failed (may just be too little motion yet)"
done

LOG "stopping container"
docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

fails=$(grep -c FAIL "$VERDICT" || true)
note "REHEARSAL $( [[ "${fails:-0}" -eq 0 ]] && echo PASSED || echo "FAILED ($fails checks)" )"
LOG "done -> $OUT (summary: rehearsal_summary.txt)"
