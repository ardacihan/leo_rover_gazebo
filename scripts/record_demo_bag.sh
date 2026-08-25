#!/usr/bin/env bash
# Record the demo evidence, on the machine that PRODUCES it.
#
#   record_demo_bag.sh leo1|leo2|laptop [outdir]
#
# Costmaps, scans and the cmd_vel chain are recorded ON the rover -- they
# must never cross the WiFi live (our own DDS traffic has starved rover
# firmware before, commit d241087). The laptop records only what already
# crosses the network: maps, the shared map, alignment, claims, TF.
#
# Stop with Ctrl+C; the bag is <outdir>/<role>_<timestamp>. Afterwards the
# bags feed the same post-hoc pipeline as the sim runs:
#   scripts/drive_replay/  (bag -> dashboard)
#   ros2 bag play + config/rviz/demo_rover_local_<ns>.rviz for costmap film.
set -eo pipefail

ROLE="$1"
OUTDIR="${2:-$HOME/demo_bags}"
[[ -n "$ROLE" ]] || { echo "usage: $0 leo1|leo2|laptop [outdir]" >&2; exit 2; }
mkdir -p "$OUTDIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$OUTDIR/${ROLE}_${STAMP}"

case "$ROLE" in
  leo1|leo2)
    ns="$ROLE"
    # Everything local to this rover that the video needs. scan_filtered and
    # the costmaps are the heavy ones -- fine on local disk, forbidden on WiFi.
    exec ros2 bag record -o "$BAG" \
      /tf /tf_static \
      /$ns/map \
      /$ns/scan_filtered \
      /$ns/wheel_odom \
      /$ns/odometry/filtered \
      /$ns/cmd_vel_nav /$ns/cmd_vel_smoothed /$ns/cmd_vel_guarded /$ns/cmd_vel \
      /$ns/local_costmap/costmap /$ns/local_costmap/published_footprint \
      /$ns/global_costmap/costmap \
      /$ns/plan \
      /$ns/tag_detections /$ns/aruco_markers \
      /$ns/shared_map /$ns/peer_transform /$ns/alignment_confidence \
      /$ns/frontier_explorer/frontiers /$ns/frontier_explorer/status
    ;;
  laptop)
    # Only network-crossing topics: this recording adds no rover load.
    exec ros2 bag record -o "$BAG" \
      /tf /tf_static \
      /leo1/map /leo2/map \
      /shared_map /shared_map_candidate \
      /leo1/shared_map /leo2/shared_map \
      /map_based_transform/leo2_to_leo1 /vetted_transform/leo2_to_leo1 \
      /alignment_confidence /vetted_alignment_confidence \
      /alignment_debug_json /alignment_locked \
      /exploration_claims \
      /leo1/tag_detections /leo2/tag_detections \
      /leo1/aruco_markers /leo2/aruco_markers \
      /leo1/frontier_explorer/frontiers /leo2/frontier_explorer/frontiers \
      /leo1/frontier_explorer/status /leo2/frontier_explorer/status
    ;;
  *)
    echo "unknown role '$ROLE' (want leo1|leo2|laptop)" >&2; exit 2 ;;
esac
