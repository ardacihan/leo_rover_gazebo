#!/usr/bin/env bash
# Start the complete two-rover simulation visibly on a native Ubuntu desktop.
# Gazebo, RViz and the browser dashboard stay running after this command exits.
#
#   scripts/run_two_robots_ubuntu.sh [world] [output-directory]
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-office_world}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-reports/live_${STAMP}/${WORLD}_coordinated}"

if [[ -d "$ROOT/.git" ]]; then
  BRANCH="$(git -C "$ROOT" branch --show-current)"
  if [[ "$BRANCH" != "feat/multi-robot-workspace" ]]; then
    echo "WARNING: active branch is '$BRANCH', expected feat/multi-robot-workspace" >&2
  fi
fi

export INTERACTIVE=true
export SIM_GUI=true
export ENABLE_CAMERA="${ENABLE_CAMERA:-true}"
# Hybrid now evaluates global grid candidates immediately, can lock a strong
# repeated grid solution before any common marker, and continuously uses real
# ArUco evidence to refine/re-anchor it. This is the same policy used on the
# physical rover path.
export ALIGN_MODE="${ALIGN_MODE:-hybrid}"
export DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"

echo "Starting two coordinated Leo rovers in $WORLD"
echo "Run output: $OUT"
exec bash "$ROOT/scripts/run_snapshot.sh" \
  "$ROOT/scripts/auto_multirobot_run.sh" coordinated "$WORLD" "$OUT" 25
