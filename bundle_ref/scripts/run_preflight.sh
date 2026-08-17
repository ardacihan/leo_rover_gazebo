#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"
WORKSPACE="$(workspace_path "${1:-}")"
PROFILE="${2:-sim_leo1}"
shift $(( $# >= 2 ? 2 : $# )) || true
source_workspace "$WORKSPACE"
exec ros2 run leo_nav2_exploration preflight_check --profile "$PROFILE" "$@"
