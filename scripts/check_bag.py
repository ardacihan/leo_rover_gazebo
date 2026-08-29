#!/usr/bin/env python3
"""Validate a rosbag2 capture before you trust it.

Every check corresponds to a way the 2026-08-25 rover-2 session produced an
unusable bag. Runs anywhere -- it reads the sqlite3 store and decodes CDR
directly, so it needs no ROS install (numpy only).

    python3 scripts/check_bag.py path/to/bag [--json report.json]

Exit code 0 = usable, 1 = at least one FAIL.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import sqlite3
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------
# expectations (rover 2, real_navigation stack)
# --------------------------------------------------------------------------
EXPECTED_HZ = {
    "/scan": 10.0,
    "/scan_filtered": 10.0,
    "/scan_uniform": 10.0,
    "/wheel_odom": 10.0,
    "/imu/data": 50.0,
    "/firmware/imu": 50.0,
    "/map": 1.0,            # slam.yaml map_update_interval: 1.0
    "/local_costmap/costmap": 5.0,
    "/global_costmap/costmap": 1.0,
}
# a topic recorded but empty is the failure mode that cost run 3
MUST_BE_NONEMPTY = ["/tf", "/tf_static", "/scan", "/scan_uniform", "/map", "/wheel_odom"]
MAX_MB_PER_S = 6.0       # above this the recorder stalls on this hardware
MAX_DEAD_FRACTION = 0.05  # graph-wide silence tolerated, as a fraction of the run
RATE_TOLERANCE = 0.6     # measured must be >= this * expected

GREEN, RED, YELLOW, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[0m"
if not sys.stdout.isatty() or os.name == "nt":
    GREEN = RED = YELLOW = RESET = ""


class Report:
    def __init__(self):
        self.rows = []
        self.failed = False

    def add(self, status, name, detail):
        self.rows.append({"status": status, "check": name, "detail": detail})
        if status == "FAIL":
            self.failed = True
        colour = {"OK": GREEN, "FAIL": RED, "WARN": YELLOW}[status]
        print(f"  {colour}{status:<4}{RESET} {name:<34} {detail}")


# --------------------------------------------------------------------------
# minimal CDR reader -- only what the checks need
# --------------------------------------------------------------------------
def _tf_transforms(buf):
    """Yield (parent, child, x, y, yaw) from a tf2_msgs/TFMessage."""
    p = 4
    n = struct.unpack_from("<I", buf, p)[0]
    p += 4
    for _ in range(n):
        p += 8                                        # header.stamp
        ln = struct.unpack_from("<I", buf, p)[0]      # frame_id
        p += 4
        parent = buf[p:p + ln - 1].decode("utf-8", "replace")
        p += ln
        p += (-(p - 4)) % 4
        ln = struct.unpack_from("<I", buf, p)[0]      # child_frame_id
        p += 4
        child = buf[p:p + ln - 1].decode("utf-8", "replace")
        p += ln
        p += (-(p - 4)) % 8
        x, y, _z, qx, qy, qz, qw = struct.unpack_from("<7d", buf, p)
        p += 56
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        yield parent, child, x, y, float(yaw)


def find_db(bag: Path) -> Path:
    if bag.is_file() and bag.suffix == ".db3":
        return bag
    dbs = sorted(bag.glob("*.db3"))
    if not dbs:
        raise SystemExit(f"no .db3 found under {bag}")
    if len(dbs) > 1:
        print(f"{YELLOW}note{RESET}: {len(dbs)} split files, checking {dbs[0].name} only")
    return dbs[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--json", help="write the report here")
    args = ap.parse_args()

    db = find_db(Path(args.bag))
    size = sum(f.stat().st_size for f in db.parent.glob("*.db3"))
    con = sqlite3.connect(f"file:{db.as_posix()}?mode=ro", uri=True)
    cur = con.cursor()
    topics = {i: n for i, n, _ in cur.execute("select id,name,type from topics")}
    by_name = {n: i for i, n in topics.items()}

    stamps = {}
    for tid, ts in cur.execute("select topic_id,timestamp from messages"):
        stamps.setdefault(tid, []).append(ts / 1e9)
    all_ts = np.sort(np.concatenate([np.array(v) for v in stamps.values()])) \
        if stamps else np.array([])
    if all_ts.size < 2:
        raise SystemExit("bag contains no messages")
    dur = float(all_ts[-1] - all_ts[0])

    rep = Report()
    print(f"\n{db.parent.name}  --  {dur:.1f} s, {sum(len(v) for v in stamps.values()):,} msgs, "
          f"{size / 1e9:.2f} GB\n")

    # -- 1. byte rate ------------------------------------------------------
    mbps = size / 1e6 / dur
    rep.add("OK" if mbps <= MAX_MB_PER_S else "FAIL", "write rate",
            f"{mbps:.1f} MB/s (limit {MAX_MB_PER_S:.0f}; raw camera is ~25)")

    # -- 2. empty topics ---------------------------------------------------
    empty = [n for n in topics.values() if not stamps.get(by_name[n])]
    critical = [n for n in empty if n in MUST_BE_NONEMPTY]
    if critical:
        rep.add("FAIL", "empty critical topics", ", ".join(critical))
    elif empty:
        rep.add("WARN", "empty topics", ", ".join(empty))
    else:
        rep.add("OK", "empty topics", "none")

    for t in MUST_BE_NONEMPTY:
        if t not in by_name:
            rep.add("FAIL", f"missing topic", f"{t} was never recorded")

    # -- 3. rates ----------------------------------------------------------
    slow = []
    for name, want in EXPECTED_HZ.items():
        tid = by_name.get(name)
        if tid is None or not stamps.get(tid):
            continue
        v = np.array(stamps[tid])
        got = (len(v) - 1) / max(v[-1] - v[0], 1e-9)
        if got < want * RATE_TOLERANCE:
            slow.append(f"{name} {got:.1f}/{want:.0f} Hz")
    rep.add("FAIL" if slow else "OK", "message rates",
            "; ".join(slow) if slow else "all within tolerance")

    # -- 4. graph-wide dead time -------------------------------------------
    gaps = np.diff(all_ts)
    dead = float(gaps[gaps > 0.4].sum())
    frac = dead / dur
    rep.add("OK" if frac <= MAX_DEAD_FRACTION else "FAIL", "recorder dead time",
            f"{dead:.1f} s / {dur:.1f} s ({frac * 100:.0f}%), "
            f"{int((gaps > 0.4).sum())} stalls, longest {gaps.max():.1f} s")

    # -- 5. SLAM actually did something ------------------------------------
    tf_id = by_name.get("/tf")
    mo = []
    if tf_id is not None:
        ids = [r[0] for r in cur.execute(
            "select id from messages where topic_id=? order by id", (tf_id,))]
        for i in range(0, len(ids), 800):
            ch = ids[i:i + 800]
            q = "select data from messages where id in (%s)" % ",".join("?" * len(ch))
            for (b,) in cur.execute(q, ch):
                for parent, child, x, y, yaw in _tf_transforms(b):
                    if parent == "map" and child == "odom":
                        mo.append((x, y, yaw))
    if not mo:
        rep.add("FAIL", "map -> odom", "never published: SLAM was not running")
    else:
        a = np.array(mo)
        moved = float(np.hypot(a[:, 0], a[:, 1]).max())
        rot = float(np.abs(a[:, 2]).max())
        if moved < 1e-6 and rot < 1e-6:
            rep.add("FAIL", "map -> odom",
                    f"identity for all {len(mo)} samples: slam_toolbox never "
                    f"scan-matched (check /scan_uniform)")
        else:
            rep.add("OK", "map -> odom",
                    f"max correction {moved:.2f} m / {np.rad2deg(rot):.1f} deg "
                    f"over {len(mo)} samples")

    # -- 6. /map freshness -------------------------------------------------
    mid = by_name.get("/map")
    if mid and stamps.get(mid):
        v = np.array(stamps[mid])
        worst = float(np.diff(v).max()) if len(v) > 1 else dur
        rep.add("OK" if worst <= 15 else "FAIL", "longest /map freeze",
                f"{worst:.0f} s ({len(v)} maps in {dur:.0f} s)")
    else:
        rep.add("FAIL", "longest /map freeze", "no /map messages at all")

    # -- 7. both costmaps present, and TF to relate them --------------------
    have_l = bool(stamps.get(by_name.get("/local_costmap/costmap")))
    have_g = bool(stamps.get(by_name.get("/global_costmap/costmap")))
    if have_l and have_g and mo:
        rep.add("OK", "costmap alignment",
                "local (odom) + global (map) + map->odom all present")
    else:
        rep.add("FAIL", "costmap alignment",
                f"local={have_l} global={have_g} map->odom={bool(mo)} "
                f"- offline alignment impossible without all three")

    # -- 8. clock agreement across machines --------------------------------
    def lag(topic, fmt_off):
        tid = by_name.get(topic)
        if tid is None:
            return None
        rows = cur.execute(
            "select timestamp,data from messages where topic_id=? limit 300", (tid,)).fetchall()
        d = []
        for ts, b in rows:
            sec, nsec = struct.unpack_from("<iI", b, fmt_off)
            d.append(sec + nsec * 1e-9 - ts / 1e9)
        return float(np.median(d)) if d else None

    sbc = lag("/wheel_odom", 4)
    jet = lag("/scan", 4)
    if sbc is not None and jet is not None:
        skew = abs(sbc - jet)
        rep.add("OK" if skew < 0.05 else "WARN", "SBC vs Jetson clock skew",
                f"{skew * 1000:.0f} ms (chrony them if > 50 ms)")

    print()
    if rep.failed:
        print(f"{RED}BAG NOT USABLE - re-record.{RESET}")
    else:
        print(f"{GREEN}bag looks good.{RESET}")
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"bag": str(db.parent), "duration_s": dur, "gb": size / 1e9,
             "checks": rep.rows, "ok": not rep.failed}, indent=1))
        print(f"report -> {args.json}")
    return 1 if rep.failed else 0


if __name__ == "__main__":
    sys.exit(main())
