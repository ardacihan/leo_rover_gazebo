#!/usr/bin/env bash
# Fetch the third-party sources this workspace needs, at the commits it was
# validated against, and apply the local world tweak.
#
# Why this exists: src/frontier_exploration_ros2, src/husarion_gz_worlds and
# src/m-explore-ros2 are nested git repositories, not submodules. Git skips
# them entirely, so `git clone`/`git pull` of this repo brings none of them --
# including husarion_gz_worlds, which contains the husarion_office world every
# multi-robot run uses. On top of that the world carries a one-line local edit
# that lived only on the machine it was made on.
#
#   ./scripts/setup_workspace.sh          # clone/checkout + patch
#   ./scripts/setup_workspace.sh --check   # verify only, change nothing
#
# Idempotent: safe to re-run. It will not touch a repo with local commits or
# uncommitted work beyond the known patch; it reports and leaves it alone.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/src"
CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

# name|url|commit validated on 2026-08-29
REPOS=(
  "frontier_exploration_ros2|https://github.com/mertgulerx/frontier_exploration_ros2.git|b0fad500e5c81ad3154f0469ca283b2702a3f90c"
  "husarion_gz_worlds|https://github.com/husarion/husarion_gz_worlds.git|0d61d49485d72ab717c557c1f7fc5baa45592c75"
  "m-explore-ros2|https://github.com/robo-friends/m-explore-ros2.git|326cf8a0b487c34246bb8f3326afbcd69576dc60"
)

fail=0
note() { printf '  %s\n' "$*"; }

for entry in "${REPOS[@]}"; do
  IFS='|' read -r name url commit <<<"$entry"
  dir="$SRC/$name"
  echo "== $name"
  if [[ ! -d "$dir/.git" ]]; then
    if (( CHECK_ONLY )); then
      note "MISSING - run without --check to clone"
      fail=1
      continue
    fi
    note "cloning $url"
    git clone --quiet "$url" "$dir"
  fi
  have="$(git -C "$dir" rev-parse HEAD)"
  if [[ "$have" != "$commit" ]]; then
    if (( CHECK_ONLY )); then
      note "at $have, expected $commit"
      fail=1
    else
      note "checking out $commit"
      git -C "$dir" fetch --quiet origin "$commit" 2>/dev/null || git -C "$dir" fetch --quiet
      git -C "$dir" checkout --quiet "$commit"
    fi
  else
    note "at $commit"
  fi
done

# ---------------------------------------------------------------- world tweak
# Gazebo's max_step_size/real_time_factor caps how fast physics may run. The
# stock world pins it to 1.0, so the simulator can never make up ground even on
# hardware that could. 2.0 lets it. On a laptop this changes nothing (the
# husarion_office runs measured 0.45x either way, bounded by the serial Gazebo
# server), but on a faster machine sim time can now outrun wall time -- so read
# the sim clock in run.log rather than assuming a wall-clock cap means minutes
# of exploration.
WORLD="$SRC/husarion_gz_worlds/worlds/husarion_office.sdf"
echo "== husarion_office real_time_factor"
if [[ ! -f "$WORLD" ]]; then
  note "world file missing"
  fail=1
elif grep -q "<real_time_factor>2.0</real_time_factor>" "$WORLD"; then
  note "already 2.0"
elif grep -q "<real_time_factor>1.0</real_time_factor>" "$WORLD"; then
  if (( CHECK_ONLY )); then
    note "still 1.0 - run without --check to patch"
    fail=1
  else
    sed -i 's|<real_time_factor>1.0</real_time_factor>|<real_time_factor>2.0</real_time_factor>|' "$WORLD"
    note "patched 1.0 -> 2.0"
  fi
else
  note "unexpected real_time_factor, leaving alone:"
  grep -n "real_time_factor" "$WORLD" | sed 's/^/    /' || true
  fail=1
fi

echo
if (( fail )); then
  echo "workspace NOT ready (see above)"
  exit 1
fi
echo "third-party sources ready. Next, inside the container:"
echo "    cd /ros2_ws && colcon build --symlink-install && source install/setup.bash"
echo "--symlink-install matters: the run harness edits src/*.py and expects"
echo "those edits to take effect without a rebuild."
