#!/usr/bin/env bash
# Run a long shell script from an immutable snapshot.
#
#   run_snapshot.sh <script> <args...>
#
# bash reads a script incrementally, by byte offset, for the whole life of the
# run. Editing that file mid-run -- which is exactly what you want to do during
# a 25-minute sim, when the run itself has just taught you something -- shifts
# every offset after the edit and bash resumes executing from the middle of a
# line. It has cost two teardowns on this branch already, both times with a
# syntax error hundreds of lines from anything I touched.
#
# So: copy the script (and it alone; it sources nothing) to a timestamped
# snapshot under .run_snapshots/ and execute that. The original stays editable.
set -eo pipefail

SCRIPT="$1"; shift
[[ -n "$SCRIPT" && -f "$SCRIPT" ]] || { echo "usage: $0 <script> [args...]" >&2; exit 2; }

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SNAPDIR="$ROOT/.run_snapshots"
mkdir -p "$SNAPDIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAP="$SNAPDIR/$(basename "$SCRIPT" .sh)_$STAMP.sh"
cp "$SCRIPT" "$SNAP"
echo "[run_snapshot] executing $SNAP (source: $SCRIPT)"
exec bash "$SNAP" "$@"
