#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "$0")/_common.sh"

WORKSPACE="$(workspace_path "${1:-}")"
FORCE=false
for arg in "${@:2}"; do
  case "$arg" in
    --force) FORCE=true ;;
    *) fail "Unknown argument: $arg" ;;
  esac
done

source_humble
require_command rosdep
require_command vcs
mkdir -p "$WORKSPACE/src"

SOURCE_PACKAGE="$BUNDLE_ROOT/src/leo_nav2_exploration"
DEST_PACKAGE="$WORKSPACE/src/leo_nav2_exploration"
[[ -f "$SOURCE_PACKAGE/package.xml" ]] || fail "Bundle package is missing: $SOURCE_PACKAGE"

if [[ "$(realpath -m "$SOURCE_PACKAGE")" != "$(realpath -m "$DEST_PACKAGE")" ]]; then
  if [[ -e "$DEST_PACKAGE" && "$FORCE" != true ]]; then
    fail "$DEST_PACKAGE already exists. Re-run with --force only after reviewing local changes."
  fi
  if [[ -e "$DEST_PACKAGE" ]]; then
    rm -rf "$DEST_PACKAGE"
  fi
  cp -a "$SOURCE_PACKAGE" "$DEST_PACKAGE"
  printf 'Copied overlay package to %s\n' "$DEST_PACKAGE"
fi

if [[ ! -d "$WORKSPACE/src/frontier_exploration_ros2/.git" ]]; then
  (cd "$WORKSPACE" && vcs import src < "$BUNDLE_ROOT/dependencies.repos")
else
  printf 'Pinned frontier_exploration_ros2 checkout already exists; leaving it unchanged.\n'
fi

rosdep install \
  --from-paths "$DEST_PACKAGE" "$WORKSPACE/src/frontier_exploration_ros2" \
  --ignore-src -r -y --rosdistro humble

printf '\nDependencies are installed. Next run:\n  %s/scripts/build_overlay.sh %q\n' "$BUNDLE_ROOT" "$WORKSPACE"
