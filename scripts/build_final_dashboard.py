#!/usr/bin/env python3
"""Build the single entry-point dashboard under final/.

Reads the three (or N) run directories under final/runs/, computes accuracy
and coverage numbers, renders the charts, and writes:

    final/index.html          the main dashboard (list of runs + comparison)
    final/<run>.html          one detail page per run
    final/assets/<run>/*.png  charts generated here

Media (maps, videos) are referenced in place under final/runs/<run>/ with
relative paths, so the whole final/ folder is self-contained and works from
the file system with no server.

Usage:
    python3 scripts/build_final_dashboard.py [--base final] [--world office_world]
"""

import argparse
import csv
import glob
import html
import json
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_multirobot_media import (  # noqa: E402
    final_locked_transform, markers_for, read_pgm)

# Validated categorical palette (dataviz reference, slots 1-4).
C_LEO1 = "#2a78d6"
C_LEO2 = "#eb6834"
C_MERGED = "#1baf7a"
RUN_COLOURS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"

def world_geometry(world):
    """Building rectangle per map frame + footprint area, from spawn_poses.

    Each rover's map is anchored on its own spawn, so the world box must be
    re-expressed per frame (clipping with raw world coordinates is the bug
    that froze every historical coverage number).
    """
    launch_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src", "leo_rover_gazebo", "launch")
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    try:
        from spawn_poses import WORLD_BOUNDS, bounds_in_robot_frame
    except ImportError:
        return None, None
    box = WORLD_BOUNDS.get(world)
    if not box:
        return None, None
    r1 = bounds_in_robot_frame(world, "leo1")
    r2 = bounds_in_robot_frame(world, "leo2")
    rects = {"leo1": r1, "leo2": r2, "shared": r1}
    area = (box[1] - box[0]) * (box[3] - box[2])
    return rects, area


def known_m2_in_rect(grid, info, rect):
    """Known cells of an int8 grid inside a rectangle of its own frame."""
    if grid is None or grid.size <= 1:
        return 0.0
    ox, oy, res = float(info[0]), float(info[1]), float(info[2])
    h, w = grid.shape
    x0, x1, y0, y1 = rect
    c0 = max(0, int((x0 - ox) / res))
    c1 = min(w, int(math.ceil((x1 - ox) / res)))
    r0 = max(0, int((y0 - oy) / res))
    r1 = min(h, int(math.ceil((y1 - oy) / res)))
    if c1 <= c0 or r1 <= r0:
        return 0.0
    sub = grid[r0:r1, c0:c1]
    return float(np.count_nonzero(sub >= 0)) * res * res


def npz_coverage_series(run, world="office_world"):
    """(t, leo1_m2, leo2_m2, shared_m2) per timelapse snapshot, clipped to
    the building rectangle in each map's own frame."""
    rects, _ = world_geometry(world)
    if rects is None:
        return []
    out = []
    for path in sorted(glob.glob(os.path.join(run, "timelapse", "snap*.npz"))):
        try:
            d = np.load(path)
            row = [float(d["t"])]
            for key in ("leo1", "leo2", "shared"):
                row.append(known_m2_in_rect(
                    d[key], d[f"{key}_info"], rects[key]))
            out.append(tuple(row))
        except (KeyError, OSError, ValueError):
            continue
    return out

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "text.color": INK,
    "font.size": 11,
    "axes.spines.top": False,
    "axes.spines.right": False,
})


# ---------------------------------------------------------------- parsing

COV_RE = re.compile(r"t=(\d+)s known=([\d.]+)m2 free=([\d.]+)m2 occ=([\d.]+)m2")


def run_times(run):
    """(start datetime, wall duration minutes) from the bag metadata."""
    import datetime
    path = os.path.join(run, "bag", "metadata.yaml")
    if not os.path.exists(path):
        return None, None
    text = open(path).read()
    m_start = re.search(
        r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)", text)
    m_dur = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if not m_start:
        return None, None
    start = datetime.datetime.fromtimestamp(int(m_start.group(1)) / 1e9)
    dur = int(m_dur.group(1)) / 1e9 / 60.0 if m_dur else None
    return start, dur


def spread_labels(ax, targets, min_frac=0.06):
    """Nudge end-of-line label y positions apart. targets: [(x, y, txt, colour)]."""
    if not targets:
        return
    lo, hi = ax.get_ylim()
    min_gap = (hi - lo) * min_frac
    order = sorted(range(len(targets)), key=lambda i: targets[i][1])
    ys = [targets[i][1] for i in order]
    for j in range(1, len(ys)):
        if ys[j] - ys[j - 1] < min_gap:
            ys[j] = ys[j - 1] + min_gap
    if ys[-1] > hi:
        ax.set_ylim(lo, ys[-1] + min_gap)
    for j, i in enumerate(order):
        x, _, txt, colour = targets[i]
        ax.annotate(txt, (x, ys[j]), color=colour, fontsize=10,
                    fontweight="bold", va="center")


def parse_coverage(path):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, errors="replace"):
        m = COV_RE.search(line)
        if m:
            out.append((int(m.group(1)), float(m.group(2))))
    return out


def parse_alignment(run):
    path = os.path.join(run, "alignment.csv")
    if not os.path.exists(path):
        return []
    with open(path) as fh:
        return list(csv.DictReader(fh))


def parse_traj(path):
    pts = []
    if not os.path.exists(path):
        return pts
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                pts.append((float(row["t"]), float(row["x"]), float(row["y"])))
            except (KeyError, ValueError):
                continue
    return pts


def traj_length(pts):
    return sum(math.hypot(x2 - x1, y2 - y1)
               for (_, x1, y1), (_, x2, y2) in zip(pts, pts[1:]))


def load_registry(run, robot):
    path = os.path.join(run, f"aruco_registry_{robot}.json")
    if not os.path.exists(path):
        return []
    try:
        data = json.load(open(path))
    except json.JSONDecodeError:
        return []
    return [(int(m["id"]), float(m["x"]), float(m["y"]), int(m.get("hits", 1)))
            for m in data.get("markers", [])]


def apply_tf(pts, tf_deg):
    """(id,x,y,hits) points through an (x, y, yaw_deg) transform."""
    x0, y0, yaw = tf_deg[0], tf_deg[1], math.radians(tf_deg[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return [(i, x0 + c * x - s * y, y0 + s * x + c * y, h)
            for i, x, y, h in pts]


def marker_accuracy(detected, truth):
    """detected [(id,x,y,hits)], truth [(id,x,y)] both in leo1/map."""
    tmap = {i: (x, y) for i, x, y in truth}
    per = []
    for i, x, y, hits in detected:
        if i in tmap:
            tx, ty = tmap[i]
            per.append({"id": i, "err_m": math.hypot(x - tx, y - ty),
                        "hits": hits, "x": x, "y": y})
    found = {p["id"] for p in per}
    missed = sorted(set(tmap) - found)
    phantom = sorted({i for i, *_ in detected} - set(tmap))
    errs = [p["err_m"] for p in per]
    return {
        "per_marker": sorted(per, key=lambda p: p["id"]),
        "missed": missed,
        "phantom": phantom,
        "mean_err_m": float(np.mean(errs)) if errs else None,
        "max_err_m": float(np.max(errs)) if errs else None,
        "n_detected": len(per),
        "n_truth": len(tmap),
    }


def read_map(run, stem):
    yaml_path = os.path.join(run, f"{stem}.yaml")
    pgm_path = os.path.join(run, f"{stem}.pgm")
    if not (os.path.exists(yaml_path) and os.path.exists(pgm_path)):
        return None
    origin = [0.0, 0.0]
    res = 0.05
    for line in open(yaml_path):
        if line.startswith("resolution"):
            res = float(line.split(":")[1])
        if line.startswith("origin"):
            origin = [float(v) for v in
                      line.split("[")[1].split("]")[0].split(",")[:2]]
    return {"img": read_pgm(pgm_path), "origin": origin, "res": res}


def count_in_log(path, pattern):
    if not os.path.exists(path):
        return 0
    rx = re.compile(pattern)
    return sum(1 for line in open(path, errors="replace") if rx.search(line))


# ---------------------------------------------------------------- charts

def chart_coverage(run, out_png, interior_m2, cov=None, label="",
                   world="office_world"):
    cov = cov if cov is not None else npz_coverage_series(run, world)
    if cov:
        series = [
            ("leo1 map", [(t, a) for t, a, _, _ in cov], C_LEO1),
            ("leo2 map", [(t, b) for t, _, b, _ in cov], C_LEO2),
            ("merged map", [(t, s) for t, _, _, s in cov], C_MERGED),
        ]
    else:
        series = [
            ("leo1 map", parse_coverage(os.path.join(run, "coverage_leo1.log")), C_LEO1),
            ("leo2 map", parse_coverage(os.path.join(run, "coverage_leo2.log")), C_LEO2),
            ("merged map", parse_coverage(os.path.join(run, "coverage.log")), C_MERGED),
        ]
    series = [(n, d, c) for n, d, c in series if d]
    if not series:
        return False
    t0 = min(d[0][0] for _, d, _ in series)
    fig, ax = plt.subplots(figsize=(8.6, 4.2))
    labels = []
    for name, data, colour in series:
        ts = [(t - t0) / 60.0 for t, _ in data]
        ys = [v for _, v in data]
        ax.plot(ts, ys, color=colour, linewidth=2)
        labels.append((ts[-1], ys[-1], f" {name}  {ys[-1]:.0f} m²", colour))
    spread_labels(ax, labels)
    if interior_m2:
        ax.axhline(interior_m2, color=MUTED, linewidth=1, linestyle="--")
        ax.annotate(f"whole building ≈ {interior_m2:.0f} m²",
                    (0.2, interior_m2), color=MUTED, fontsize=9,
                    va="bottom")
    ax.set_xlabel("minutes of exploration")
    ax.set_ylabel("mapped area (m²)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.margins(x=0.14)
    if label:
        ax.set_title(label, fontsize=11, color=INK2, loc="left")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return True


def chart_alignment(run, out_png, label=""):
    rows = parse_alignment(run)
    pts = []
    for r in rows:
        try:
            t = float(r["t"])
            err = float(r["err_xy_m"]) if r.get("err_xy_m") else math.nan
            yaw = (float(r["err_yaw_deg"])
                   if r.get("err_yaw_deg") else math.nan)
            locked = r.get("locked", "0").strip() == "1"
            pts.append((t, err, yaw, locked))
        except ValueError:
            continue
    if not pts:
        return False
    t0 = pts[0][0]
    ts = [(t - t0) / 60.0 for t, *_ in pts]
    errs = [e for _, e, _, _ in pts]
    yaws = [abs(y) for _, _, y, _ in pts]
    locked = [lk for *_, lk in pts]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.6, 5.2), sharex=True)
    for ax in (ax1, ax2):
        on = None
        for i, lk in enumerate(locked):
            if lk and on is None:
                on = ts[i]
            if (not lk or i == len(locked) - 1) and on is not None:
                ax.axvspan(on, ts[i], color=C_MERGED, alpha=0.10, lw=0)
                on = None
    ax1.plot(ts, errs, color=C_LEO1, linewidth=2)
    ax1.set_ylabel("position error (m)")
    ax1.set_ylim(bottom=0)
    ax2.plot(ts, yaws, color=C_LEO1, linewidth=2)
    ax2.set_ylabel("rotation error (°)")
    ax2.set_xlabel("minutes of exploration")
    ax2.set_ylim(bottom=0)
    ax1.set_title(f"{label}Merge alignment vs the true spawn offset "
                  "(green = merge active)", fontsize=11, color=INK2,
                  loc="left")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return True


def chart_markers(run, out_png, detected, truth, title, base_map="merged_map"):
    m = read_map(run, base_map) or read_map(run, "leo1_map")
    if m is None:
        return False
    img = m["img"]
    ox, oy = m["origin"]
    res = m["res"]
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]
    fig, ax = plt.subplots(figsize=(8.6, 6.2))
    ax.imshow(np.flipud(img), cmap="gray", vmin=0, vmax=254,
              extent=extent, origin="lower")
    tx = [x for _, x, y in truth]
    ty = [y for _, x, y in truth]
    ax.scatter(tx, ty, marker="s", s=90, facecolors="none",
               edgecolors=C_MERGED, linewidths=2, label="true position")
    for i, x, y in truth:
        ax.annotate(str(i), (x, y), xytext=(6, 6),
                    textcoords="offset points", color=C_MERGED, fontsize=9,
                    fontweight="bold")
    if detected:
        dx = [x for _, x, y, _ in detected]
        dy = [y for _, x, y, _ in detected]
        ax.scatter(dx, dy, marker="x", s=70, color=C_LEO2, linewidths=2,
                   label="where the robot placed it")
        tmap = {i: (x, y) for i, x, y in truth}
        for i, x, y, _ in detected:
            if i in tmap:
                ax.plot([x, tmap[i][0]], [y, tmap[i][1]], color=C_LEO2,
                        linewidth=1, alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)
    ax.set_title(title, fontsize=11, color=INK2, loc="left")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return True


def chart_compare_coverage(runs, out_png, world="office_world"):
    fig, ax = plt.subplots(figsize=(8.6, 4.2))
    drew = False
    labels = []
    for (name, run), colour in zip(runs, RUN_COLOURS):
        started, _ = run_times(run)
        when = f" ({started:%H:%M})" if started else ""
        name = f"{name}{when}"
        cov = npz_coverage_series(run, world)
        data = ([(t, s) for t, _, _, s in cov] if cov
                else parse_coverage(os.path.join(run, "coverage.log")))
        if not data:
            continue
        t0 = data[0][0]
        ts = [(t - t0) / 60.0 for t, _ in data]
        ys = [v for _, v in data]
        ax.plot(ts, ys, color=colour, linewidth=2)
        labels.append((ts[-1], ys[-1], f" {name}  {ys[-1]:.0f} m²", colour))
        drew = True
    if not drew:
        plt.close(fig)
        return False
    spread_labels(ax, labels)
    ax.set_xlabel("minutes of exploration")
    ax.set_ylabel("merged map size (m²)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.margins(x=0.16)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return True


# ---------------------------------------------------------------- stats

def run_stats(run, world):
    _, interior = world_geometry(world)
    cov = npz_coverage_series(run, world)
    if cov:
        cov1 = [(t, a) for t, a, _, _ in cov]
        cov2 = [(t, b) for t, _, b, _ in cov]
        cov_merged = [(t, s) for t, _, _, s in cov]
    else:
        cov_merged = parse_coverage(os.path.join(run, "coverage.log"))
        cov1 = parse_coverage(os.path.join(run, "coverage_leo1.log"))
        cov2 = parse_coverage(os.path.join(run, "coverage_leo2.log"))
    # The saved merged map is the authoritative final state (the last
    # snapshot can predate the map saver by a few seconds).
    final_map = read_map(run, "merged_map")
    final_merged = None
    rects, _ = world_geometry(world)
    if final_map is not None and rects is not None:
        grid = np.where(final_map["img"] == 205, -1, 0).astype(np.int8)
        final_merged = known_m2_in_rect(
            np.flipud(grid),
            (final_map["origin"][0], final_map["origin"][1], final_map["res"]),
            rects["shared"])
    align = parse_alignment(run)

    lock_t = None
    locked_rows = 0
    final_err = final_yaw = None
    t0 = float(align[0]["t"]) if align else None
    for r in align:
        if r.get("locked", "0").strip() == "1":
            locked_rows += 1
            if lock_t is None:
                lock_t = float(r["t"]) - t0
            if r.get("err_xy_m"):
                try:
                    final_err = float(r["err_xy_m"])
                    final_yaw = float(r["err_yaw_deg"])
                except ValueError:
                    pass

    geometry = None
    gpath = os.path.join(run, "geometry_score.json")
    if os.path.exists(gpath):
        try:
            g = json.load(open(gpath))
            geometry = g[0] if isinstance(g, list) else g
        except (json.JSONDecodeError, IndexError):
            pass

    validation = None
    vpath = os.path.join(run, "office_validation.json")
    if os.path.exists(vpath):
        try:
            v = json.load(open(vpath))
            validation = v[0] if isinstance(v, list) else v
        except (json.JSONDecodeError, IndexError):
            pass

    truth = markers_for(world)
    tf = final_locked_transform(run)
    reg1 = load_registry(run, "leo1")
    reg2 = load_registry(run, "leo2")
    reg2_common = apply_tf(reg2, tf) if tf else []
    merged_reg = {}
    for i, x, y, h in reg1 + reg2_common:
        if i not in merged_reg or h > merged_reg[i][3]:
            merged_reg[i] = (i, x, y, h)
    acc1 = marker_accuracy(reg1, truth)
    acc2 = marker_accuracy(reg2_common, truth) if reg2_common else None
    accm = marker_accuracy(list(merged_reg.values()), truth)

    d1 = traj_length(parse_traj(os.path.join(run, "traj_leo1.csv")))
    d2 = traj_length(parse_traj(os.path.join(run, "traj_leo2.csv")))

    explorer_log = os.path.join(run, "explorer.log")
    merged_m2 = final_merged if final_merged else (
        cov_merged[-1][1] if cov_merged else None)
    started, wall_min = run_times(run)
    stats = {
        "started": started,
        "wall_min": wall_min,
        "final_merged_m2": merged_m2,
        "final_leo1_m2": cov1[-1][1] if cov1 else None,
        "final_leo2_m2": cov2[-1][1] if cov2 else None,
        "coverage_pct": (100.0 * merged_m2 / interior
                         if merged_m2 and interior else None),
        "duration_min": ((cov_merged[-1][0] - cov_merged[0][0]) / 60.0
                         if len(cov_merged) > 1 else None),
        "lock_after_min": lock_t / 60.0 if lock_t is not None else None,
        "locked_pct": 100.0 * locked_rows / len(align) if align else None,
        "final_err_xy_m": final_err,
        "final_err_yaw_deg": final_yaw,
        "geometry": geometry,
        "validation": validation,
        "tf": tf,
        "acc_leo1": acc1,
        "acc_leo2": acc2,
        "acc_merged": accm,
        "truth": truth,
        "reg1": reg1,
        "reg2_common": reg2_common,
        "merged_reg": list(merged_reg.values()),
        "dist_leo1_m": d1,
        "dist_leo2_m": d2,
        "finished": count_in_log(explorer_log, r"Exploration finished\."),
        "aborted": count_in_log(explorer_log, r"Exploration aborted:"),
        "blacklists": count_in_log(explorer_log, r"blacklisting goal"),
        "long_bans": count_in_log(explorer_log, r"long-banning"),
        "planner_aborts": count_in_log(
            os.path.join(run, "nav2.log"), r"failed to create plan"),
    }
    return stats


# ---------------------------------------------------------------- insights

def fmt(v, unit="", nd=1):
    if v is None:
        return "—"
    return f"{v:.{nd}f}{unit}"


def auto_insights(name, s):
    """Plain-language findings computed from the numbers."""
    out = []
    if s["coverage_pct"] is not None:
        pct = s["coverage_pct"]
        if pct >= 85:
            out.append(f"The pair mapped {pct:.0f}% of the building — "
                       "close to complete.")
        elif pct >= 60:
            out.append(
                f"The pair mapped {pct:.0f}% of the building. The missing "
                "part is usually a room whose doorway goal kept failing — "
                "check the exploration videos to see which one.")
        else:
            out.append(
                f"Only {pct:.0f}% of the building was mapped. Something "
                "stopped exploration early — look at the per-robot videos "
                "and the event counts below.")
    if s["lock_after_min"] is None:
        out.append(
            "The two maps NEVER merged in this run: the aligner never "
            "accepted a transform. The 'merged' map is just leo1's map. "
            "The usual cause is that the maps did not overlap enough, or "
            "every candidate failed a quality gate.")
    else:
        out.append(
            f"The maps first merged after {s['lock_after_min']:.1f} min and "
            f"the merge was active {s['locked_pct']:.0f}% of the time.")
        if s["final_err_xy_m"] is not None:
            e = s["final_err_xy_m"]
            if e <= 0.15:
                out.append(f"Final merge offset vs truth: {e * 100:.0f} cm — "
                           "very good.")
            elif e <= 0.40:
                out.append(
                    f"Final merge offset vs truth: {e * 100:.0f} cm. Walls "
                    "will look slightly doubled where the two maps overlap. "
                    "This is mostly SLAM drift inside each robot's own map, "
                    "not a bad merge: no rigid shift can fix a bent map.")
            else:
                out.append(
                    f"Final merge offset vs truth: {e * 100:.0f} cm — the "
                    "merge locked onto a shifted answer. Expect visibly "
                    "doubled walls.")
    g = s["geometry"]
    if g:
        verdict = "passed" if g.get("ok") else "FAILED"
        out.append(
            f"Independent wall-overlap check on the saved maps: {verdict} "
            f"(typical wall gap {fmt(g.get('median_m'), ' m', 3)}, worst 10% "
            f"{fmt(g.get('p90_m'), ' m', 3)}).")
    accm = s["acc_merged"]
    if accm and accm["n_detected"]:
        out.append(
            f"Markers: {accm['n_detected']}/{accm['n_truth']} found, "
            f"average placement error {fmt(accm['mean_err_m'], ' m', 2)}"
            + (f", missed {accm['missed']}" if accm["missed"] else "")
            + (f", false detections {accm['phantom']}" if accm["phantom"]
               else "") + ".")
    if s["aborted"]:
        out.append(
            f"{s['aborted']} robot(s) ABORTED exploration (stuck or never "
            "reached a goal) — that part of the building stayed unknown.")
    if s["planner_aborts"] > 150:
        out.append(
            f"The path planner failed {s['planner_aborts']} times — almost "
            "always at narrow doorways. Fewer failures than ~150 is normal; "
            "this many means a doorway was effectively impassable for a "
            "while, and goals behind it got banned.")
    if s["blacklists"] > 6:
        out.append(
            f"{s['blacklists']} goals were banned after the robot stalled "
            "on the way. Banned areas come back after a timeout now, but "
            "many bans still slow a run down.")
    return out


# ---------------------------------------------------------------- html

PAGE_CSS = """
:root { color-scheme: light; }
* { box-sizing: border-box; }
body { margin: 0; background: #f9f9f7; color: #0b0b0b;
       font: 14px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif; }
.wrap { max-width: 1080px; margin: 0 auto; padding: 24px 20px 80px; }
h1 { font-size: 26px; margin: 8px 0 4px; }
h2 { font-size: 19px; margin: 36px 0 10px; }
h3 { font-size: 15px; margin: 20px 0 8px; color: #52514e; }
.sub { color: #52514e; margin: 0 0 18px; }
.card { background: #fcfcfb; border: 1px solid rgba(11,11,11,0.10);
        border-radius: 10px; padding: 18px 20px; margin: 14px 0; }
.runlist a.runcard { display: block; text-decoration: none; color: inherit; }
.runlist .card:hover { border-color: #2a78d6; }
.runlist .title { font-size: 17px; font-weight: 600; color: #2a78d6; }
table { border-collapse: collapse; width: 100%; margin: 10px 0; }
th, td { text-align: left; padding: 6px 10px;
         border-bottom: 1px solid #e1e0d9; font-variant-numeric: tabular-nums; }
th { color: #52514e; font-weight: 600; }
tr.bad td { color: #d03b3b; }
img.fig { max-width: 100%; border: 1px solid rgba(11,11,11,0.10);
          border-radius: 8px; background: #fcfcfb; }
.runtag { display: inline-block; padding: 3px 12px; border-radius: 6px;
          color: #fff; font-weight: 700; font-size: 14px; margin-right: 10px; }
video { max-width: 100%; border-radius: 8px; background: #000; }
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
@media (max-width: 800px) { .grid2 { grid-template-columns: 1fr; } }
.pill { display: inline-block; padding: 2px 10px; border-radius: 999px;
        font-size: 12px; font-weight: 600; margin-right: 6px; }
.pill.ok   { background: #e3f2e3; color: #006300; }
.pill.warn { background: #fdeeda; color: #8a5a00; }
.pill.bad  { background: #fbe3e3; color: #a02c2c; }
ul.insights li { margin: 7px 0; }
a { color: #2a78d6; }
.back { font-size: 13px; }
.note { color: #52514e; font-size: 13px; }
"""


def esc(t):
    return html.escape(str(t))


def pill(ok, ok_txt, bad_txt, warn=False):
    if ok:
        return f'<span class="pill ok">{esc(ok_txt)}</span>'
    cls = "warn" if warn else "bad"
    return f'<span class="pill {cls}">{esc(bad_txt)}</span>'


def fig_block(base_dir, rel, label, why_missing=""):
    """<img> when the figure exists, an honest note when it does not."""
    if os.path.exists(os.path.join(base_dir, rel)):
        return (f'<div><h3>{esc(label)}</h3>'
                f'<img class="fig" loading="lazy" src="{esc(rel)}"></div>')
    note = why_missing or "not produced for this run"
    return (f'<div><h3>{esc(label)}</h3>'
            f'<p class="note">{esc(note)}</p></div>')


def video_block(rel, label):
    return (f'<div><h3>{esc(label)}</h3>'
            f'<video controls preload="metadata" src="{esc(rel)}"></video>'
            f'</div>')


def marker_table(acc):
    if acc is None:
        return "<p class='note'>No usable detections.</p>"
    rows = "".join(
        f"<tr><td>{p['id']}</td><td>{p['err_m']:.2f} m</td>"
        f"<td>{p['hits']}</td></tr>" for p in acc["per_marker"])
    extra = ""
    if acc["missed"]:
        extra += (f"<p class='note'>Not seen at all: markers "
                  f"{', '.join(map(str, acc['missed']))}.</p>")
    if acc["phantom"]:
        extra += (f"<p class='note'>Detected but not real (false "
                  f"positives): {', '.join(map(str, acc['phantom']))}.</p>")
    return (f"<table><tr><th>marker</th><th>placement error</th>"
            f"<th>sightings</th></tr>{rows}</table>{extra}")


def build_run_page(base, name, run_rel, s, assets_rel, insights_extra="",
                   colour="#2a78d6"):
    run = os.path.join(base, run_rel)
    base_dir = base
    base = base_dir
    med = lambda f: os.path.join(run_rel, f)  # noqa: E731
    have = lambda f: os.path.exists(os.path.join(run, f))  # noqa: E731

    ins = "".join(f"<li>{esc(t)}</li>" for t in auto_insights(name, s))

    g = s["geometry"] or {}
    v = s["validation"] or {}
    merge_ok = s["lock_after_min"] is not None
    when = (f"{s['started']:%a %b %d, %H:%M}" if s.get("started") else "—")
    dur = fmt(s.get("wall_min"), " min wall", 0)
    body = f"""
<title>{esc(name)} — run detail</title>
<style>{PAGE_CSS}</style>
<div class="wrap">
<p class="back"><a href="index.html">← all runs</a></p>
<h1><span class="runtag" style="background:{colour}">{esc(name)}</span>
{esc(when)}</h1>
<p class="sub">Two rovers exploring the office together. Started {esc(when)},
recorded {dur}. Everything on this page belongs to <b>{esc(name)}</b> only.</p>

<div class="card">
{pill(merge_ok, "maps merged", "maps never merged")}
{pill(s["finished"] >= 2, "both robots finished", f"{s['finished']}/2 finished", warn=True)}
{pill(bool(g.get("ok")), "wall-overlap check passed", "wall-overlap check failed", warn=True)}
</div>

<h2>What happened (plain language)</h2>
<div class="card"><ul class="insights">{ins}</ul>{insights_extra}</div>

<h2>The numbers</h2>
<div class="card">
<table>
<tr><th>metric</th><th>value</th><th>what it means</th></tr>
<tr><td>building mapped</td><td>{fmt(s["coverage_pct"], "%", 0)}</td>
    <td>share of the world footprint known in the merged map</td></tr>
<tr><td>merged map size</td><td>{fmt(s["final_merged_m2"], " m²", 0)}</td>
    <td>leo1: {fmt(s["final_leo1_m2"], " m²", 0)}, leo2: {fmt(s["final_leo2_m2"], " m²", 0)}</td></tr>
<tr><td>first merge</td><td>{fmt(s["lock_after_min"], " min")}</td>
    <td>time until the two maps were first stitched together</td></tr>
<tr><td>merge active</td><td>{fmt(s["locked_pct"], "%", 0)}</td>
    <td>share of the run with an accepted map-to-map transform</td></tr>
<tr><td>merge offset vs truth</td><td>{fmt(s["final_err_xy_m"], " m", 2)} / {fmt(s["final_err_yaw_deg"], "°")}</td>
    <td>how far the accepted stitch is from the true spawn offset</td></tr>
<tr><td>wall gap after merge</td><td>{fmt(g.get("median_m"), " m", 3)} (median), {fmt(g.get("p90_m"), " m", 3)} (p90)</td>
    <td>distance between the same wall seen by both robots</td></tr>
<tr><td>distance driven</td><td>leo1 {fmt(s["dist_leo1_m"], " m", 0)}, leo2 {fmt(s["dist_leo2_m"], " m", 0)}</td>
    <td></td></tr>
<tr><td>path-planner failures</td><td>{s["planner_aborts"]}</td>
    <td>mostly doorway goals the planner could not route to</td></tr>
<tr><td>banned goals</td><td>{s["blacklists"]} (+{s["long_bans"]} long bans)</td>
    <td>goals given up after repeated failures</td></tr>
</table>
</div>

<h2>Maps and paths</h2>
{fig_block(base, med('traj_overlay.png'),
           "Merged map with both robots' paths (blue = leo1, orange = leo2, stars = true marker spots)")}
<div class="grid2">
{fig_block(base, med('leo1_map.png'), "leo1's own map")}
{fig_block(base, med('leo2_map.png'), "leo2's own map",
           "single-robot run - there is no second robot")}
{fig_block(base, med('merged_map.png'), "Merged map (clean, no paths)",
           "single-robot run - nothing to merge")}
{fig_block(base, med('candidate_map.png'), "Candidate merge (what the aligner was considering)",
           "no candidate merge in this run")}
</div>

<h2>Marker accuracy</h2>
<p class="note">Squares are where the markers really are; orange crosses are
where each robot placed them on its map. Shorter connecting lines = better.</p>
<div class="grid2">
{fig_block(base, f"{assets_rel}/markers_leo1.png", "leo1's markers")}
{fig_block(base, f"{assets_rel}/markers_leo2.png", "leo2's markers (moved into the shared frame)",
           "no second robot, or its markers could not be placed in the shared frame")}
</div>
{fig_block(base, f"{assets_rel}/markers_merged.png", "Merged marker registry")}
<div class="grid2">
<div><h3>leo1 table</h3>{marker_table(s["acc_leo1"])}</div>
<div><h3>leo2 table</h3>{marker_table(s["acc_leo2"])}</div>
</div>
<div><h3>merged table</h3>{marker_table(s["acc_merged"])}</div>

<h2>Charts</h2>
{fig_block(base, f"{assets_rel}/coverage.png", "Mapped area over time")}
{fig_block(base, f"{assets_rel}/alignment.png", "Merge accuracy over time",
           "this run had no merge to measure: either a single robot, or the "
           "relative position was known from the start so nothing was estimated")}

<h2>Videos</h2>
<p class="note">All videos run from the start of the run to the end.</p>
<div class="grid2">
{video_block(med('camera_leo1.mp4'), "leo1 camera — full run") if have('camera_leo1.mp4') else ''}
{video_block(med('camera_leo2.mp4'), "leo2 camera — full run") if have('camera_leo2.mp4') else ''}
{video_block(med('map_explore_leo1.mp4'), "leo1 exploring — its map growing") if have('map_explore_leo1.mp4') else ''}
{video_block(med('map_explore_leo2.mp4'), "leo2 exploring — its map growing") if have('map_explore_leo2.mp4') else ''}
</div>
{video_block(med('map_explore_merged.mp4'), "Merged map — both robots together") if have('map_explore_merged.mp4') else ''}
</div>
"""
    out = os.path.join(base, f"{name}.html")
    open(out, "w").write(body)
    return out


def list_bundles(bundles_dir):
    """[(id, meta)] for every sibling bundle, newest session first."""
    out = []
    for d in glob.glob(os.path.join(bundles_dir, "*")):
        meta_path = os.path.join(d, "bundle.json")
        if os.path.isdir(d) and os.path.exists(meta_path):
            try:
                out.append((os.path.basename(d), json.load(open(meta_path))))
            except json.JSONDecodeError:
                continue
    out.sort(key=lambda b: b[1].get("started", ""), reverse=True)
    return out


def bundle_dropdown(bundles, current_id, prefix):
    """<select> that navigates between run sessions."""
    if not bundles:
        return ""
    opts = ""
    for bid, meta in bundles:
        sel = " selected" if bid == current_id else ""
        opts += (f'<option value="{prefix}{bid}/index.html"{sel}>'
                 f'{esc(meta.get("title", bid))}</option>')
    return (
        '<label class="note" style="display:block;margin:8px 0 16px">'
        'Run session:&nbsp;'
        f'<select style="font:inherit;padding:4px 8px" '
        f'onchange="if(this.value)location.href=this.value">{opts}</select>'
        "</label>")


def build_index(base, entries, world, overall_extra="", title="",
                nav_html=""):
    cards = ""
    for idx, (name, s) in enumerate(entries):
        merge_ok = s["lock_after_min"] is not None
        colour = RUN_COLOURS[idx % len(RUN_COLOURS)]
        when = (f"{s['started']:%a %b %d, %H:%M}" if s.get("started") else "")
        dur = fmt(s.get("wall_min"), " min", 0)
        cards += f"""
<a class="runcard" href="{name}.html"><div class="card">
<div class="title"><span class="runtag" style="background:{colour}">{esc(name)}</span>
{esc(when)} &middot; {dur} →</div>
<p style="margin:6px 0 10px">
{pill(merge_ok, "maps merged", "maps never merged")}
{pill(s["finished"] >= 2, "both robots finished", f"{s['finished']}/2 finished", warn=True)}
{pill(bool((s["geometry"] or {}).get("ok")), "wall check ok", "wall check failed", warn=True)}
</p>
<table>
<tr><th>building mapped</th><th>merge offset</th><th>markers found</th><th>driven</th></tr>
<tr><td>{fmt(s["coverage_pct"], "%", 0)}</td>
<td>{fmt(s["final_err_xy_m"], " m", 2)}</td>
<td>{(s["acc_merged"] or {}).get("n_detected", "—")}/{(s["acc_merged"] or {}).get("n_truth", "—")}</td>
<td>{fmt((s["dist_leo1_m"] or 0) + (s["dist_leo2_m"] or 0), " m", 0)}</td></tr>
</table>
</div></a>"""

    rows = ""
    for name, s in entries:
        g = s["geometry"] or {}
        when = (f"{s['started']:%b %d %H:%M}" if s.get("started") else "—")
        rows += (f"<tr><td><a href='{name}.html'>{esc(name)}</a></td>"
                 f"<td>{esc(when)}</td>"
                 f"<td>{fmt(s['coverage_pct'], '%', 0)}</td>"
                 f"<td>{fmt(s['lock_after_min'], ' min')}</td>"
                 f"<td>{fmt(s['final_err_xy_m'], ' m', 2)}</td>"
                 f"<td>{fmt(g.get('median_m'), ' m', 3)}</td>"
                 f"<td>{fmt((s['acc_merged'] or {}).get('mean_err_m'), ' m', 2)}</td>"
                 f"<td>{s['finished']}/2</td>"
                 f"<td>{s['planner_aborts']}</td></tr>")

    n = len(entries)
    body = f"""
<title>{esc(title or "Leo Rover Runs")}</title>
<style>{PAGE_CSS}</style>
<div class="wrap">
<h1>{esc(title or "Two-robot exploration — results")}</h1>
{nav_html}
<p class="sub">{n} run{"s" if n != 1 else ""} in this session, on the same
office map ({esc(world)}). Click a run for its full page: maps, marker
accuracy, charts, and full videos. The summary below belongs to this
session only.</p>

<div class="runlist">{cards}</div>

<h2>Side-by-side</h2>
<div class="card">
<table>
<tr><th>run</th><th>when</th><th>mapped</th><th>first merge</th><th>merge offset</th>
<th>wall gap</th><th>marker error</th><th>finished</th><th>planner fails</th></tr>
{rows}
</table>
<h3>Merged map growth, all runs</h3>
<img class="fig" loading="lazy" src="assets/comparison_coverage.png">
</div>

<h2>What we learned</h2>
<div class="card">{overall_extra or "<p class='note'>Analysis pending.</p>"}</div>
</div>
"""
    open(os.path.join(base, "index.html"), "w").write(body)


# ---------------------------------------------------------------- main

def build_root_index(final_root):
    """The landing page: pick a run session (bundle)."""
    bundles = list_bundles(os.path.join(final_root, "bundles"))
    if not bundles:
        return
    nav = bundle_dropdown(bundles, bundles[0][0], "bundles/")
    cards = ""
    for bid, meta in bundles:
        runs = meta.get("runs", [])
        rows = "".join(
            f"<tr><td>{esc(r['name'])}</td>"
            f"<td>{esc(r.get('mapped', '—'))}</td>"
            f"<td>{esc(r.get('merge', '—'))}</td></tr>" for r in runs)
        cards += f"""
<a class="runcard" href="bundles/{bid}/index.html"><div class="card">
<div class="title">{esc(meta.get("title", bid))} →</div>
<p class="note" style="margin:4px 0 8px">{len(runs)} run{"s" if len(runs) != 1 else ""}
&middot; {esc(meta.get("note", ""))}</p>
<table><tr><th>run</th><th>mapped</th><th>merge</th></tr>{rows}</table>
</div></a>"""
    body = f"""
<title>Leo Rover Run Sessions</title>
<style>{PAGE_CSS}</style>
<div class="wrap">
<h1>Two-robot exploration — run sessions</h1>
{nav}
<p class="sub">Each session is a bundle of runs done together, with its own
summary, charts and videos. Pick one from the dropdown or the cards.</p>
<div class="runlist">{cards}</div>
</div>
"""
    open(os.path.join(final_root, "index.html"), "w").write(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="final")
    ap.add_argument("--world", default="office_world")
    ap.add_argument("--title", default="",
                    help="display name of this run session (bundle)")
    ap.add_argument("--note", default="",
                    help="one-line description shown on the session card")
    args = ap.parse_args()
    base = args.base

    run_dirs = sorted(
        d for d in glob.glob(os.path.join(base, "runs", "run*"))
        if os.path.isdir(d))
    if not run_dirs:
        print(f"no run dirs under {base}/runs", file=sys.stderr)
        return 1

    entries = []
    named = []
    for idx, run in enumerate(run_dirs):
        name = os.path.basename(run)
        named.append((name, run))
        s = run_stats(run, args.world)
        colour = RUN_COLOURS[idx % len(RUN_COLOURS)]
        tag = (f"{name} \u00b7 {s['started']:%b %d %H:%M} \u2014 "
               if s.get("started") else f"{name} \u2014 ")
        assets = os.path.join(base, "assets", name)
        os.makedirs(assets, exist_ok=True)
        chart_coverage(run, os.path.join(assets, "coverage.png"),
                       world_geometry(args.world)[1],
                       label=f"{tag}mapped area over time",
                       world=args.world)
        chart_alignment(run, os.path.join(assets, "alignment.png"), label=tag)
        chart_markers(run, os.path.join(assets, "markers_leo1.png"),
                      s["reg1"], s["truth"],
                      f"{tag}leo1's detections vs true marker spots")
        chart_markers(run, os.path.join(assets, "markers_leo2.png"),
                      s["reg2_common"], s["truth"],
                      f"{tag}leo2's detections (in the shared frame) vs truth")
        chart_markers(run, os.path.join(assets, "markers_merged.png"),
                      s["merged_reg"], s["truth"],
                      f"{tag}merged registry vs truth")

        extra_path = os.path.join(base, "insights", f"{name}.html")
        extra = open(extra_path).read() if os.path.exists(extra_path) else ""
        build_run_page(base, name, os.path.join("runs", name), s,
                       f"assets/{name}", insights_extra=extra, colour=colour)
        entries.append((name, s))
        print(f"built {name}.html")

    os.makedirs(os.path.join(base, "assets"), exist_ok=True)
    chart_compare_coverage(
        named, os.path.join(base, "assets", "comparison_coverage.png"),
        world=args.world)
    overall_path = os.path.join(base, "insights", "overall.html")
    overall = open(overall_path).read() if os.path.exists(overall_path) else ""

    # Bundle bookkeeping: sessions live under final/bundles/<id>/ and every
    # page in a session can jump to the others through the dropdown.
    bundles_dir = os.path.dirname(os.path.abspath(base))
    bundle_id = os.path.basename(os.path.abspath(base))
    is_bundle = os.path.basename(bundles_dir) == "bundles"
    title = args.title
    nav = ""
    if is_bundle:
        first_start = min(
            (s["started"] for _, s in entries if s.get("started")),
            default=None)
        if not title:
            old_meta = os.path.join(base, "bundle.json")
            if os.path.exists(old_meta):
                title = json.load(open(old_meta)).get("title", "")
        if not title:
            kind = "in parallel" if len(entries) > 1 else "solo"
            title = (f"{first_start:%b %d %H:%M} — {len(entries)} "
                     f"run{'s' if len(entries) != 1 else ''} {kind}"
                     if first_start else bundle_id)
        note = args.note
        if not note:
            old_meta = os.path.join(base, "bundle.json")
            if os.path.exists(old_meta):
                note = json.load(open(old_meta)).get("note", "")
        meta = {
            "title": title,
            "note": note,
            "started": f"{first_start}" if first_start else "",
            "runs": [{
                "name": name,
                "mapped": fmt(s["coverage_pct"], "%", 0),
                "merge": (fmt(s["final_err_xy_m"], " m off", 2)
                          if s["lock_after_min"] is not None
                          else "never merged"),
            } for name, s in entries],
        }
        json.dump(meta, open(os.path.join(base, "bundle.json"), "w"),
                  indent=1)
        nav = bundle_dropdown(list_bundles(bundles_dir), bundle_id, "../")

    build_index(base, entries, args.world, overall_extra=overall,
                title=title, nav_html=nav)
    print(f"built {base}/index.html")
    if is_bundle:
        refresh_sibling_navs(bundles_dir, bundle_id)
        build_root_index(os.path.dirname(bundles_dir))
        print(f"built {os.path.dirname(bundles_dir)}/index.html")
    return 0


def refresh_sibling_navs(bundles_dir, current_id):
    """A new session must appear in the dropdowns of the older sessions."""
    bundles = list_bundles(bundles_dir)
    rx = re.compile(
        r'<label class="note" style="display:block;margin:8px 0 16px">'
        r'.*?</label>', re.S)
    for bid, _ in bundles:
        if bid == current_id:
            continue
        page = os.path.join(bundles_dir, bid, "index.html")
        if not os.path.exists(page):
            continue
        html_text = open(page).read()
        nav = bundle_dropdown(bundles, bid, "../")
        new_text, n = rx.subn(nav, html_text, count=1)
        if n:
            open(page, "w").write(new_text)


if __name__ == "__main__":
    raise SystemExit(main())
