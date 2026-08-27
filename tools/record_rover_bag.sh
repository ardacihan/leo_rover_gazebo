#!/usr/bin/env bash
# Record a real-rover run without filling the Jetson's disk.
#
#   tools/record_rover_bag.sh <name> [lean|full] [outdir]
#
#   lean (default)  everything but depth      ~25 MB/min
#   full            + throttled depth          ~48 MB/min
#
# Both assume debug_color_throttle.py at its default 2 Hz. At HZ=5 colour
# alone is 47 MB/min; see tools/README.md for the arithmetic.
#
# Run it ON the rover, with the stack already up, from a terminal that has
# `source /opt/ros/humble/setup.bash` and the right ROS_DOMAIN_ID.
# Start tools/debug_color_throttle.py FIRST (this script checks and warns).
# Stop with Ctrl+C -- rosbag2 writes metadata.yaml on SIGINT only.
#
# Where the size actually goes, measured on drive_2026-08-20 (723 MB / 9.6 min,
# i.e. 75 MB/min, one file, no throttling):
#
#     338 MB  48%   /bag/depth/compressed        190 kB/msg @ ~3 Hz
#     313 MB  44%   /bag/color/compressed        157 kB/msg @ ~3.5 Hz
#      23 MB   3%   /scan                          4 kB/msg @ 10 Hz
#      13 MB   2%   the two camera_info topics     0.4 kB/msg @ 30 Hz (!)
#      21 MB   3%   everything else -- odom, TF, IMU, cmd chain, wheel states
#
# So: 92% is the two camera streams, and the whole robot-state picture -- the
# part you actually need for a map, a path, a costmap film or a safety audit --
# costs about 6 MB/min. The levers, in order:
#
#   1. Drop depth unless you are replaying the camera costmap layer. Halves it.
#   2. Record the DRIVER'S jpeg, never the raw Image. A raw 640x480 rgb8 frame
#      is 921 kB against ~157 kB for the same picture as jpeg -- 4.6 MB/s at
#      5 Hz. debug_color_throttle.py forwards the driver's own jpeg untouched.
#   3. Throttle camera_info. At 30 Hz it costs more than /cmd_vel + /tf + IMU
#      combined, and a replay needs one message per second at most.
#   4. zstd the bag. It does nothing for jpeg/png payloads but roughly halves
#      scans, costmaps, odometry and TF.
#   5. Split the file (--max-bag-size). A 4 GB single .db3 is a bad thing to
#      have to scp off a Jetson.
#
# Never recorded here, on purpose:
#   /cmd_vel_raw   the safety gate audits its subscribers and CLOSES on an
#                  unexpected one ("unexpected raw command consumers:
#                  rosbag2_recorder"). Recording it stops the robot.
#   raw images     bagging 640x480 color+depth raw pushed load average to
#                  9.3 on 6 cores and opened 0.4-0.5 s scan arrival gaps that
#                  tripped the explorer watchdogs.
set -eo pipefail

NAME="${1:-}"
PROFILE="${2:-lean}"
OUTDIR="${3:-$HOME/leo_bags}"
[[ -n "$NAME" ]] || { echo "usage: $0 <name> [lean|full] [outdir]" >&2; exit 2; }
case "$PROFILE" in lean|full) ;; *) echo "profile must be lean|full" >&2; exit 2 ;; esac

SPLIT_MB="${SPLIT_MB:-1024}"
COMPRESS="${COMPRESS:-zstd}"
mkdir -p "$OUTDIR"
BAG="$OUTDIR/${NAME}_$(date +%Y%m%d_%H%M%S)"

have() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

if ! have /bag/color/compressed; then
  echo "WARNING: /bag/color/compressed is not published."
  echo "         Start it first, in its own terminal:"
  echo "           python3 tools/debug_color_throttle.py"
  echo "         Recording anyway -- the bag will have no camera."
  sleep 3
fi

# The topic set that answers "what did the robot see, decide and do", minus the
# two topics that must never be recorded. Kept flat and explicit rather than
# regex-matched: a --regex that accidentally catches /cmd_vel_raw closes the
# gate mid-run, and the failure looks like a hardware fault.
TOPICS=(
  # perception
  /scan /scan_lidar_base /scan_collision_fused /scan_slam_fused
  /camera/scan_collision /camera/scan_slam
  # the map and the path -- the presentation artifacts
  /map /map_metadata /exploration_path /tf /tf_static
  # the command chain, for the safety audit (request -> gate -> firmware)
  /cmd_vel_request /cmd_vel /rob_2/cmd_vel
  /collision_monitor/approach_footprint /collision_monitor/footprint
  /collision_monitor/transition_event
  # odometry and firmware health
  /wheel_odom /merged_odom /odometry/filtered
  /rob_2/firmware/wheel_odom /rob_2/firmware/imu
  /rob_2/firmware/battery_averaged
  # logs -- the explorer's reasoning is only in rosout
  /rosout
  # ArUco. Tiny MarkerArrays, and the reason a second run can be merged onto
  # this one at all. NOT /aruco/debug_image -- that is a raw Image.
  # The registry JSON is a FILE, not a topic: the offline aligner reads
  # aruco_registry_<name>.json, so tools/finish_run.sh copies it next to the
  # bag. A bag alone cannot be merged.
  /aruco_markers /aruco_detections /aruco_markers_poses
  # camera, throttled and already jpeg. These names are what
  # scripts/drive_replay/ reads -- a bag under other names replays as
  # pictures only, with no costmap/plan/frontier reconstruction.
  # /bag/color/camera_info is what makes OFFLINE ArUco possible: the detector
  # needs K and D to solvePnP. Without it the bag holds pictures you cannot
  # turn into marker poses, and the lab session cannot be redone.
  /bag/color/compressed /bag/color/camera_info /rob_4/camera/depth/camera_info
)

if [[ "$PROFILE" == "full" ]]; then
  # Depth is half the bytes and there is exactly one reason to pay them: the
  # offline costmap replay. drive_replay/depth_to_points.py rebuilds the
  # RealSense cloud from this topic so Nav2's camera ObstacleLayer runs as it
  # does on hardware. Needs DEPTH=1 on debug_color_throttle.py (its default).
  TOPICS+=(/bag/depth/compressed)
fi

# Drop topics that do not exist on this rover rather than letting rosbag2 sit
# waiting for them -- namespaces have differed between jetson-02 and -04.
KEEP=()
for t in "${TOPICS[@]}"; do
  if have "$t"; then KEEP+=("$t"); else echo "note: $t absent, skipping"; fi
done
[[ ${#KEEP[@]} -gt 0 ]] || { echo "FATAL: none of the topics exist -- is the stack up?" >&2; exit 1; }

CFLAGS=()
if [[ "$COMPRESS" != "none" ]]; then
  if ros2 pkg list 2>/dev/null | grep -q rosbag2_compression_zstd; then
    CFLAGS=(--compression-mode message --compression-format "$COMPRESS")
  else
    echo "note: rosbag2_compression_zstd not installed; recording uncompressed"
  fi
fi

echo "recording ${#KEEP[@]} topics -> $BAG (profile=$PROFILE, split=${SPLIT_MB}MB)"
echo "Ctrl+C to stop. Do NOT kill -9: metadata.yaml is written on SIGINT."
exec ros2 bag record -o "$BAG" \
  "${CFLAGS[@]}" \
  --max-bag-size $((SPLIT_MB * 1024 * 1024)) \
  "${KEEP[@]}"
