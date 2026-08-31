#!/usr/bin/env bash
# Two fully independent office experiments at once.
# Each experiment is Leo1+Leo2 on its own ROS domain and Gazebo partition.
# They are not "two robots"; they are two isolated two-robot runs.
#
#   scripts/run_parallel_office.sh [cap_min]
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAP="${1:-14}"
STAMP=$(date +%Y-%m-%d_%H%M%S)
BASE="reports/validation_${STAMP}"
OUT1="$BASE/office_run1"
OUT2="$BASE/office_run2"
LOG() { echo "[parallel $(date +%H:%M:%S)] $*"; }

mkdir -p "$ROOT/$OUT1" "$ROOT/$OUT2" "$ROOT/$BASE"
export MAX_WALL_MIN="${MAX_WALL_MIN:-35}"
# The linux launcher default tag is leo_rover_humble:bundle; this host
# builds leo_rover_humble:latest. Honour an explicit override.
export LEO_IMAGE="${LEO_IMAGE:-leo_rover_humble:latest}"

LOG "Run 1 -> $OUT1 (container leo_sim_r1, domain 31)"
CONTAINER_NAME=leo_sim_r1 ROS_DOMAIN_ID=31 IGN_PARTITION=leo_r1 GZ_PARTITION=leo_r1 \
  bash "$ROOT/scripts/auto_multirobot_run.sh" coordinated office_world "$OUT1" "$CAP" \
  > "$ROOT/$OUT1/host.log" 2>&1 &
PID1=$!

LOG "Run 2 -> $OUT2 (container leo_sim_r2, domain 32)"
CONTAINER_NAME=leo_sim_r2 ROS_DOMAIN_ID=32 IGN_PARTITION=leo_r2 GZ_PARTITION=leo_r2 \
  bash "$ROOT/scripts/auto_multirobot_run.sh" coordinated office_world "$OUT2" "$CAP" \
  > "$ROOT/$OUT2/host.log" 2>&1 &
PID2=$!

set +e
wait "$PID1"; RC1=$?
wait "$PID2"; RC2=$?
set -e

LOG "Run 1 exit=$RC1  Run 2 exit=$RC2"
python3 "$ROOT/scripts/score_merge_geometry.py" "$ROOT/$OUT1" "$ROOT/$OUT2" \
  | tee "$ROOT/$BASE/geometry_score.json" || true
python3 "$ROOT/scripts/validate_office_run.py" "$ROOT/$OUT1" "$ROOT/$OUT2" \
  | tee "$ROOT/$BASE/office_validation.json" || true
python3 "$ROOT/scripts/validate_office_run.py" "$ROOT/$OUT1" \
  > "$ROOT/$OUT1/office_validation.json" || true
python3 "$ROOT/scripts/validate_office_run.py" "$ROOT/$OUT2" \
  > "$ROOT/$OUT2/office_validation.json" || true
# Dashboards: use scripts/build_final_dashboard.py against a base dir with
# runs/ inside it (the old playback dashboard builder was removed).
echo "run1=$RC1 run2=$RC2" > "$ROOT/$BASE/summary.txt"
# A geometry fail is a failed validation even if explorers finished.
python3 - <<PY
import json, sys
try:
    data = json.load(open("$ROOT/$BASE/geometry_score.json"))
except Exception:
    sys.exit(1)
rows = data if isinstance(data, list) else [data]
try:
    office = json.load(open("$ROOT/$BASE/office_validation.json"))
    office = office if isinstance(office, list) else [office]
except Exception:
    office = []
sys.exit(0 if all(r.get("ok") for r in rows) and len(office) == 2
         and all(r.get("ok") for r in office)
         and $RC1 == 0 and $RC2 == 0 else 3)
PY
