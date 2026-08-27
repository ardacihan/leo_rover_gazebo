#!/usr/bin/env bash
# ONE script for the presentation demo: bring the stack up, start a small
# rosbag plus the light-weight recorders, then hand the rover to teleop.
#
#   scripts/demo_teleop_record.sh <world> [num_robots] [outdir]
#
#   world:       husarion_office | office_world | depot_world | big_world
#   num_robots:  1 (default) or 2
#   outdir:      repo-relative, default reports/demo_<world>_<timestamp>
#
# Then, in a SECOND terminal:   scripts/demo_teleop_wsl.sh [num_robots]
# Stop with Ctrl+C here; the maps, plots and bag are finalised on the way out.
#
# What lands in <outdir>:
#   bag/                 rosbag2 -- costmaps, lidar, TF, cmd_vel, throttled map,
#                        low-res JPEG video. Tens of MB for a 10 min drive,
#                        not GB (see "Why it stays small" below).
#   leoN_map.pgm/.yaml   the final map(s)
#   coverage_leoN.log    known area vs sim time, sampled every 15 s
#   coverage.png         the coverage-over-time plot
#   traj_leoN.csv        the robot path over time (t,robot,x,y)
#   traj_overlay.png     path drawn on the final map
#   timelapse/*.npz      map-over-time snapshots, re-renderable offline
#
# Why it stays small -- the four levers, in order of how much they save:
#   1. The camera is never bagged raw. demo_bag_feeds.py republishes it as
#      320 px JPEG at 4 Hz (~12 kB/frame vs 920 kB/frame at 640x480 rgb8).
#   2. /leoN/map is bagged through the same node at 0.2 Hz, not slam_toolbox's
#      1 Hz -- a full OccupancyGrid every 5 s instead of every second.
#   3. Message-level zstd on the bag. Occupancy grids and costmaps are mostly
#      runs of -1/0/100 and compress about 10x.
#   4. Nothing depth, nothing point-cloud, no /leoN/aruco/debug_image.
# Override with VIDEO_W / VIDEO_HZ / MAP_HZ / JPEG_Q / BAG_COMPRESS=none.
set -eo pipefail

WORLD="${1:-husarion_office}"
NUM_ROBOTS="${2:-1}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${3:-reports/demo_${WORLD}_$(date +%Y%m%d_%H%M%S)}"

VIDEO_W="${VIDEO_W:-320}"
VIDEO_HZ="${VIDEO_HZ:-4.0}"
JPEG_Q="${JPEG_Q:-60}"
MAP_HZ="${MAP_HZ:-0.2}"
BAG_COMPRESS="${BAG_COMPRESS:-zstd}"
DURATION_MIN="${DURATION_MIN:-0}"          # 0 = drive until Ctrl+C
# Ground-truth odometry is the honest thing to switch OFF (it hands SLAM a
# perfect prior no rover has), but it is also what makes a live demo reliable.
# The default matches the measured runs; GT_ODOM=true for a safe demo.
GT_ODOM="${GT_ODOM:-false}"

case "$NUM_ROBOTS" in
  1|2) ;;
  *) echo "FATAL: num_robots must be 1 or 2 (the aligner/merger is pairwise)" >&2
     exit 2 ;;
esac

mkdir -p "$ROOT/$OUT"
LOG() { echo "[demo $(date +%H:%M:%S)] $*" | tee -a "$ROOT/$OUT/run.log"; }

in_sim()    { docker exec    leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }
in_sim_bg() { docker exec -d leo_sim bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && $1"; }

# Same world clip the measured runs use, so a demo curve is comparable to them.
case "$WORLD" in
  office_world)     BOUNDS="-12 12 -8 8" ;;
  depot_world)      BOUNDS="-7.5 7.5 -7.5 7.5" ;;
  husarion_office)  BOUNDS="-4 27 -15 4" ;;
  *)                BOUNDS="" ;;
esac

# ---------- teardown, on Ctrl+C or on the duration cap ----------
finish() {
  trap - INT TERM
  echo
  LOG "finishing: flushing bag and recorders"
  # SIGINT, not SIGKILL: rosbag2 writes metadata.yaml on SIGINT only. A killed
  # recording leaves a .db3 with no metadata, which ros2 bag play refuses.
  in_sim 'pkill -INT -f "ros2 bag record" || true' || true
  in_sim 'pkill -INT -f "map_coverage[.]py" || true; pkill -INT -f "traj_recorder[.]py" || true; pkill -INT -f "merge_timelapse_recorder[.]py" || true' || true
  sleep 8

  for i in $(seq 1 "$NUM_ROBOTS"); do
    ns="leo$i"
    LOG "saving final map: $ns"
    in_sim "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/$OUT/${ns}_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -r map:=/$ns/map" \
      >> "$ROOT/$OUT/map_saver.log" 2>&1 || LOG "  map_saver_cli ($ns) failed, see map_saver.log"
  done

  LOG "rendering plots (coverage over time, path over map)"
  in_sim "python3 /ros2_ws/scripts/render_multirobot_media.py /ros2_ws/$OUT --world $WORLD --title 'demo $WORLD'" \
    >> "$ROOT/$OUT/render.log" 2>&1 || LOG "  render_multirobot_media failed, see render.log"
  # Also the standalone single-run curve, which needs no world raster.
  in_sim "python3 /ros2_ws/scripts/plot_coverage.py /ros2_ws/$OUT/coverage_leo1.log /ros2_ws/$OUT/coverage_leo1.png 'demo $WORLD leo1'" \
    >> "$ROOT/$OUT/render.log" 2>&1 || true

  LOG "bag size: $(in_sim "du -sh /ros2_ws/$OUT/bag 2>/dev/null | cut -f1" || echo '?')"
  LOG "artifacts in $OUT:"
  ls -1 "$ROOT/$OUT" | sed 's/^/    /' | tee -a "$ROOT/$OUT/run.log"
  LOG "replay: ros2 bag play /ros2_ws/$OUT/bag --clock  +  rviz2 -d config/rviz/demo_rover_local_leo1.rviz"
  LOG "leaving leo_sim running; stop it with: docker stop leo_sim"
  exit 0
}

# ---------- 1. sim ----------
LOG "stopping any previous sim container"
docker stop leo_sim >/dev/null 2>&1 || true
docker rm leo_sim >/dev/null 2>&1 || true

LOG "starting sim: world=$WORLD robots=$NUM_ROBOTS camera=on gt_odom_tf=$GT_ODOM"
if [[ "$NUM_ROBOTS" -eq 2 ]]; then
  LOG "  NOTE: two rendering cameras is the load that has segfaulted Gazebo"
  LOG "        inside the WSL D3D12 driver at 9-13 min. Keep 2-rover demos short."
fi
WORLD="$WORLD" GUI=false NUM_ROBOTS="$NUM_ROBOTS" ENABLE_CAMERA=true \
  GT_ODOM_TF="$GT_ODOM" "$ROOT/scripts/sim_gpu_wsl.sh" >>"$ROOT/$OUT/run.log" 2>&1

LAST_NS="leo${NUM_ROBOTS}"
LOG "waiting for /$LAST_NS/scan"
ok=""
for _ in $(seq 1 48); do
  if in_sim "ros2 topic list 2>/dev/null | grep -q '^/$LAST_NS/scan\$'"; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || { LOG "FATAL: scan topic never appeared"; docker logs --tail 60 leo_sim >>"$ROOT/$OUT/run.log" 2>&1; exit 1; }

# From here on a Ctrl+C must still collect artifacts.
trap finish INT TERM

# ---------- 2. realistic odometry (unless GT_ODOM=true) ----------
if [[ "$GT_ODOM" != "true" ]]; then
  LOG "starting realistic odometry (wheel + degraded gyro -> EKF), one per rover"
  for i in $(seq 1 "$NUM_ROBOTS"); do
    ns="leo$i"
    in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_odom.py --ros-args -p use_sim_time:=true -p input_topic:=/$ns/odom -p output_topic:=/$ns/odom_wheel_like -p odom_frame:=$ns/odom -p base_frame:=$ns/base_link -p publish_tf:=false -p zero_origin:=true -p seed:=$i > /ros2_ws/$OUT/odom_$ns.log 2>&1"
    in_sim_bg "exec python3 /ros2_ws/scripts/sim_realism_imu.py --ros-args -p use_sim_time:=true -p input_topic:=/$ns/imu/data -p output_topic:=/$ns/imu/data_real -p seed:=$i > /ros2_ws/$OUT/imu_$ns.log 2>&1"
    in_sim_bg "exec ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/$ns --params-file /ros2_ws/scripts/ekf_$ns.yaml -r /tf:=/tf -r /tf_static:=/tf_static > /ros2_ws/$OUT/ekf_$ns.log 2>&1"
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
fi

# ---------- 3. SLAM + Nav2 ----------
# slam_multi/nav2_multi even for one rover: they namespace everything as
# /leo1/..., so the topic names, the RViz configs and the bag topic list are
# identical whether the demo runs one rover or two.
LOG "starting SLAM (slam_multi, num_robots=$NUM_ROBOTS)"
in_sim_bg "exec ros2 launch leo_rover_gazebo slam_multi.launch.py num_robots:=$NUM_ROBOTS > /ros2_ws/$OUT/slam.log 2>&1"
sleep 12
LOG "starting Nav2 (nav2_multi) -- this is what publishes the costmaps"
in_sim_bg "exec ros2 launch leo_rover_gazebo nav2_multi.launch.py num_robots:=$NUM_ROBOTS > /ros2_ws/$OUT/nav2.log 2>&1"

LOG "waiting for the costmaps"
ok=""
for _ in $(seq 1 40); do
  n=$(in_sim "ros2 topic list 2>/dev/null | grep -c '/local_costmap/costmap\$' || true")
  n=${n//[^0-9]/}; n=${n:-0}
  if [[ "$n" -ge "$NUM_ROBOTS" ]] 2>/dev/null; then ok=1; break; fi
  sleep 5
done
[[ -n "$ok" ]] || LOG "WARNING: costmaps never appeared; the bag will have no costmap topics"

# ---------- 4. bootstrap jog ----------
# slam_toolbox publishes no map at all until the rover has moved, so the first
# 30 s of a demo would otherwise look broken on the projector.
LOG "bootstrap jog (slam_toolbox needs motion before it publishes a map)"
in_sim "for i in \$(seq 1 $NUM_ROBOTS); do (timeout 6 ros2 topic pub -r 5 /leo\$i/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}' >/dev/null 2>&1 &) ; done; sleep 7; for i in \$(seq 1 $NUM_ROBOTS); do ros2 topic pub --once /leo\$i/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; done"

# ---------- 5. cheap feeds + light recorders ----------
LOG "starting demo feeds (${VIDEO_W}px jpeg @ ${VIDEO_HZ} Hz, map @ ${MAP_HZ} Hz)"
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/demo_bag_feeds.py --ros-args -p use_sim_time:=true -p robot:=$ns -p width:=$VIDEO_W -p image_hz:=$VIDEO_HZ -p map_hz:=$MAP_HZ -p jpeg_quality:=$JPEG_Q > /ros2_ws/$OUT/feeds_$ns.log 2>&1"
done

LOG "starting recorders (coverage, trajectory, map time-lapse)"
ROBOTS=$(seq -s, 1 "$NUM_ROBOTS" | sed 's/\([0-9]\)/leo\1/g')
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  in_sim_bg "exec python3 /ros2_ws/scripts/map_coverage.py 15 $BOUNDS /$ns/map > /ros2_ws/$OUT/coverage_$ns.log 2>&1"
  in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ns /ros2_ws/$OUT/traj_$ns.csv 2.0 $ns/map > /ros2_ws/$OUT/traj_$ns.log 2>&1"
done
# render_multirobot_media.py reads traj.csv for the overlay; for a single-rover
# demo that is just leo1's track in its own map frame.
in_sim_bg "exec python3 /ros2_ws/scripts/traj_recorder.py $ROBOTS /ros2_ws/$OUT/traj.csv 2.0 leo1/map > /ros2_ws/$OUT/traj.log 2>&1"
in_sim_bg "exec python3 /ros2_ws/scripts/merge_timelapse_recorder.py /ros2_ws/$OUT/timelapse 5 > /ros2_ws/$OUT/timelapse.log 2>&1"

# ---------- 6. the bag ----------
TOPICS="/clock /tf /tf_static"
for i in $(seq 1 "$NUM_ROBOTS"); do
  ns="leo$i"
  TOPICS="$TOPICS /$ns/demo/map /$ns/demo/image/compressed"
  TOPICS="$TOPICS /$ns/scan /$ns/odom /$ns/odometry/filtered /$ns/cmd_vel"
  TOPICS="$TOPICS /$ns/local_costmap/costmap /$ns/local_costmap/costmap_updates"
  TOPICS="$TOPICS /$ns/local_costmap/published_footprint"
  TOPICS="$TOPICS /$ns/global_costmap/costmap /$ns/global_costmap/costmap_updates"
  TOPICS="$TOPICS /$ns/plan"
done

COMPRESS=""
if [[ "$BAG_COMPRESS" != "none" ]]; then
  if in_sim "ros2 pkg list 2>/dev/null | grep -q rosbag2_compression_zstd"; then
    COMPRESS="--compression-mode message --compression-format $BAG_COMPRESS"
  else
    LOG "WARNING: rosbag2_compression_zstd not installed; recording uncompressed"
  fi
fi
LOG "recording bag -> $OUT/bag"
echo "ros2 bag record -o /ros2_ws/$OUT/bag $COMPRESS $TOPICS" > "$ROOT/$OUT/bag_cmdline.txt"
in_sim_bg "exec ros2 bag record -o /ros2_ws/$OUT/bag $COMPRESS $TOPICS > /ros2_ws/$OUT/bag.log 2>&1"
sleep 5

# ---------- 7. drive ----------
cat <<MSG | tee -a "$ROOT/$OUT/run.log"

  ============================================================
   RECORDING. Now open a SECOND terminal and drive:

     ./run_demo_teleop.ps1                    (Windows)
     scripts/demo_teleop_wsl.sh $NUM_ROBOTS              (WSL)

   Watch it live (third terminal, needs a display):
     rviz2 -d config/rviz/demo_rover_local_leo1.rviz

   Ctrl+C HERE when you are done -- maps, plots and the bag
   are finalised on the way out.
  ============================================================

MSG

deadline=0
if [[ "$DURATION_MIN" -gt 0 ]] 2>/dev/null; then
  deadline=$(( $(date +%s) + DURATION_MIN * 60 ))
fi
while true; do
  sleep 30
  cov=$(grep -oE 'known=[0-9.]+m2' "$ROOT/$OUT/coverage_leo1.log" 2>/dev/null | tail -1 || true)
  size=$(in_sim "du -sh /ros2_ws/$OUT/bag 2>/dev/null | cut -f1" 2>/dev/null || true)
  LOG "  coverage=${cov:-waiting}  bag=${size:-0}"
  if ! docker ps --format '{{.Names}}' | grep -qx leo_sim; then
    LOG "FATAL: container died"
    finish
  fi
  if [[ "$deadline" -gt 0 && $(date +%s) -ge "$deadline" ]]; then
    LOG "duration cap of ${DURATION_MIN} min reached"
    finish
  fi
done
