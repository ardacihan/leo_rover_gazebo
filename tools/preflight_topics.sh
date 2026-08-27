#!/usr/bin/env bash
# Prove every topic is actually FLOWING before you start recording.
#
#   tools/preflight_topics.sh [lean|full]
#
# A topic existing in `ros2 topic list` proves nothing: ROS discovery and the
# CLI daemon both retain stale names, and a node that crashed at startup leaves
# its advertisements behind. Every "the bag was missing data" ends here — the
# recorder subscribed to a name that was never going to deliver a message, and
# said nothing about it for the whole run.
#
# So this measures. For each topic: is there a publisher, and did a message
# arrive inside the timeout. Anything that fails is named, with the usual cause.
set -eo pipefail

PROFILE="${1:-full}"
TIMEOUT="${TIMEOUT:-6}"

declare -a MISSING=() SILENT=() OK=()

check() {                       # check <topic> <why-it-matters>
  local t="$1" why="$2"
  if ! ros2 topic list 2>/dev/null | grep -qx "$t"; then
    MISSING+=("$t|$why"); printf '  %-46s MISSING\n' "$t"; return
  fi
  # `ros2 topic echo --once` blocks forever on a topic with a publisher that
  # never publishes, which is exactly the failure being hunted -- hence timeout.
  if timeout "$TIMEOUT" ros2 topic echo "$t" --once >/dev/null 2>&1; then
    OK+=("$t"); printf '  %-46s ok\n' "$t"
  else
    SILENT+=("$t|$why"); printf '  %-46s SILENT (%ss)\n' "$t" "$TIMEOUT"
  fi
}

echo "=== cannot be regenerated offline: if it is not here, it is lost ==="
check /scan                       "lidar - no scan, no map, no costmap replay"
check /tf                         "map<-odom<-base; every pose depends on it"
check /tf_static                  "sensor mounts; markers land wrong without it"
check /bag/color/compressed       "camera - start tools/debug_color_throttle.py"
check /bag/color/camera_info      "K and D - without these NO offline ArUco"
check /wheel_odom                 "odometry"
check /rob_2/firmware/imu         "gyro"
check /rob_2/firmware/battery_averaged "battery; also the liveness canary"
if [[ "$PROFILE" == "full" ]]; then
  check /bag/depth/compressed     "depth - without it no camera costmap layer"
  check /rob_4/camera/depth/camera_info "depth intrinsics for the cloud rebuild"
fi

echo
echo "=== live products: nice to have, all reconstructable from the above ==="
check /map                        "SLAM output"
check /scan_collision_fused       "fused scan"
check /odometry/filtered          "EKF"

echo
echo "=== disk ==="
# A full disk is the other way a bag ends up "missing data": rosbag2 keeps
# running and keeps failing to write. 10 min at 848x480/5 Hz is ~0.9 GB.
avail_kb=$(df -Pk "${HOME}" | awk 'NR==2 {print $4}')
avail_gb=$(( avail_kb / 1024 / 1024 ))
printf '  %-46s %s GB free\n' "${HOME}" "$avail_gb"
if [[ "$avail_gb" -lt 5 ]]; then
  echo "  WARNING: under 5 GB. Two legs will not fit. Clear ~/leo_bags first."
fi

echo
if [[ ${#MISSING[@]} -eq 0 && ${#SILENT[@]} -eq 0 ]]; then
  echo "ALL CLEAR - ${#OK[@]} topics flowing. Start the bag."
  exit 0
fi

echo "NOT READY. Fix these before recording:"
for e in "${MISSING[@]}"; do
  echo "  MISSING  ${e%%|*}  -- ${e##*|}"
done
for e in "${SILENT[@]}"; do
  echo "  SILENT   ${e%%|*}  -- ${e##*|}"
done
cat <<'HINT'

Usual causes, in the order they actually happen:
  /bag/color/*        debug_color_throttle.py not started, or the driver
                      publishes no .../compressed (image_transport plugins
                      missing). ros2 topic list | grep compressed
  /scan SILENT        lidar service down, or it advertises and never spins:
                      systemctl restart lidar
  /tf missing map     SLAM has not published a map->odom yet. It will not
                      until the robot has MOVED. Nudge it and re-run this.
  firmware topics     the SBC link is starved. Close the rover web UI tab,
                      then tools/firmware_stability_monitor.py.
  everything SILENT   wrong ROS_DOMAIN_ID in this shell.
HINT
exit 1
