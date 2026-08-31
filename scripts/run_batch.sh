#!/usr/bin/env bash
# N simultaneous coordinated two-rover runs on one world, into one bundle.
#
#   run_batch.sh <world> <bundle_dir> [n_runs] [cap_min]
#
# Each run gets its own container, ROS domain and Gazebo partition. Outputs
# land in <bundle_dir>/runs/run{1..N}. Finalize afterwards with:
#   BASE=<bundle_dir>/runs WORLD=<world> bash scripts/finalize_runs.sh
set -u
cd "$(dirname "$0")/.."

WORLD=${1:?usage: run_batch.sh <world> <bundle_dir> [n_runs] [cap_min]}
BUNDLE=${2:?usage: run_batch.sh <world> <bundle_dir> [n_runs] [cap_min]}
N=${3:-3}
CAP_MIN=${4:-30}

export LEO_IMAGE=leo_rover_humble:latest
export MAX_WALL_MIN=${MAX_WALL_MIN:-50}
export SIM_SPEED=${SIM_SPEED:-2.0}

pids=()
for i in $(seq 1 "$N"); do
  mkdir -p "$BUNDLE/runs/run$i"
  CONTAINER_NAME=leo_sim_r$i ROS_DOMAIN_ID=$((30 + i)) \
  IGN_PARTITION=leo_r$i GZ_PARTITION=leo_r$i \
    bash scripts/auto_multirobot_run.sh coordinated "$WORLD" "$BUNDLE/runs/run$i" "$CAP_MIN" \
    > "$BUNDLE/runs/run$i/host.log" 2>&1 &
  pids+=($!)
  sleep 20   # stagger startup so gazebo instances don't spike together
done

rc=0
for i in $(seq 0 $((N - 1))); do
  wait "${pids[$i]}"; r=$?
  echo "run$((i + 1)) exit=$r"
  [ "$r" -ne 0 ] && rc=1
done
echo "batch $WORLD done rc=$rc"
exit $rc
