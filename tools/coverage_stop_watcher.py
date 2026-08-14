#!/usr/bin/env python3
"""Stop the explorer automatically when the map is (nearly) fully covered.

Tails the safe_mapping launch log for map_coverage_reporter lines and sends
SIGINT to the explorer's process group when coverage has converged, defined
as N consecutive reports with no reachable frontiers (the reporter prints
one report every ~5 s). The explorer's shutdown path publishes repeated zero
commands, so this is a graceful mission end, not a kill.

Usage (on the Jetson):
    python3 coverage_stop_watcher.py <safemap_log> <explorer_pid_file> \
        [consecutive_reports] [min_runtime_s]

Defaults: 4 consecutive frontier-free reports (~20 s), 90 s minimum runtime
before a stop is allowed (a young map is briefly frontier-free while small).
"""

import os
import re
import signal
import sys
import time

LOG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/claude_safemap5.log"
PIDFILE = sys.argv[2] if len(sys.argv) > 2 else "/tmp/claude_explorer.pid"
NEEDED = int(sys.argv[3]) if len(sys.argv) > 3 else 4
MIN_RUNTIME = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0

start = time.time()
streak = 0
pos = os.path.getsize(LOG)
print(f"watching {LOG}; will SIGINT pgid from {PIDFILE} after "
      f"{NEEDED} consecutive frontier-free reports (min runtime "
      f"{MIN_RUNTIME:.0f}s)", flush=True)

while True:
    time.sleep(2.0)
    try:
        with open(LOG, "r", errors="replace") as f:
            f.seek(pos)
            chunk = f.read()
            pos = f.tell()
    except OSError:
        continue
    for line in chunk.splitlines():
        if "map_coverage_reporter" not in line:
            continue
        if "no reachable frontiers" in line:
            streak += 1
            print(f"frontier-free report {streak}/{NEEDED}", flush=True)
        elif re.search(r"frontier:\s+\d+ cells", line) or "frontier clusters" in line:
            if "0 frontier clusters" in line:
                streak += 1
                print(f"frontier-free report {streak}/{NEEDED}", flush=True)
            elif re.search(r"\|\s+[1-9]\d* frontier clusters", line):
                streak = 0
    if streak >= NEEDED and time.time() - start >= MIN_RUNTIME:
        try:
            with open(PIDFILE) as f:
                pid = int(f.read().strip())
            os.killpg(pid, signal.SIGINT)
            print(f"coverage converged; sent SIGINT to pgid {pid}", flush=True)
        except (OSError, ValueError) as exc:
            print(f"failed to signal explorer: {exc}", flush=True)
        break
