#!/usr/bin/env python3
"""Render per-robot exploration-map videos plus a merged-map video from the
timelapse .npz snapshots, as browser-playable H.264 MP4s.

Usage:
    python3 scripts/render_map_videos.py <run_dir> [--fps 8] [--size 900]

Writes into <run_dir>:
    map_explore_leo1.mp4   leo1's own map growing, with its path and goal
    map_explore_leo2.mp4   same for leo2
    map_explore_merged.mp4 the accepted merge with both paths
Every snapshot is used (no subsampling) so the videos cover the whole run.
"""

import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_timelapse import panel, world_to_px, LEO1, LEO2  # noqa: E402
from render_multirobot_media import markers_for  # noqa: E402

GOAL = (60, 200, 255)   # BGR amber
TRUTH = (122, 175, 27)  # BGR of #1baf7a - true marker positions
DET = (10, 113, 232)    # BGR orange - detected marker positions


def truth_markers_in_frame(world, robot):
    """True marker positions converted into one robot's map frame."""
    in_leo1 = markers_for(world)  # (id, x, y) in leo1/map
    if robot == "leo1":
        return in_leo1
    try:
        launch_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "src", "leo_rover_gazebo", "launch")
        if launch_dir not in sys.path:
            sys.path.insert(0, launch_dir)
        from spawn_poses import SPAWN_POSES
        p1 = SPAWN_POSES[world]["leo1"]
        p2 = SPAWN_POSES[world][robot]
    except (ImportError, KeyError):
        return []
    # leo1/map -> world -> robot/map
    s1x, s1y, _, _, _, yaw1 = (float(v) for v in p1)
    s2x, s2y, _, _, _, yaw2 = (float(v) for v in p2)
    out = []
    for mid, x, y in in_leo1:
        c, s = math.cos(yaw1), math.sin(yaw1)
        wx, wy = s1x + c * x - s * y, s1y + s * x + c * y
        c, s = math.cos(-yaw2), math.sin(-yaw2)
        out.append((mid, c * (wx - s2x) - s * (wy - s2y),
                    s * (wx - s2x) + c * (wy - s2y)))
    return out


def letterbox_mapper(shape, panel_size):
    """panel() letterboxes the map; map grid pixels to panel pixels."""
    h, w = shape
    scale = min(panel_size[0] / w, panel_size[1] / h)
    x0 = (panel_size[0] - max(1, int(w * scale))) // 2
    y0 = (panel_size[1] - max(1, int(h * scale))) // 2
    return lambda p: (int(x0 + p[0] * scale), int(y0 + p[1] * scale))


def draw_markers(img, info, shape, panel_size, truth, detected):
    to_panel = letterbox_mapper(shape, panel_size)
    for mid, x, y in truth:
        p = to_panel(world_to_px(x, y, info, shape))
        cv2.rectangle(img, (p[0] - 5, p[1] - 5), (p[0] + 5, p[1] + 5),
                      TRUTH, 2)
        cv2.putText(img, str(mid), (p[0] + 7, p[1] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TRUTH, 1, cv2.LINE_AA)
    for mid, x, y in detected:
        p = to_panel(world_to_px(x, y, info, shape))
        cv2.drawMarker(img, p, DET, cv2.MARKER_TILTED_CROSS, 11, 2,
                       cv2.LINE_AA)


def detections_of(d, ns):
    key_ids, key_pos = f"{ns}_tagids", f"{ns}_tagpos"
    if key_ids not in d.files or key_pos not in d.files:
        return []
    ids, pos = d[key_ids], d[key_pos]
    if pos.size < 2:
        return []
    return [(int(i), float(p[0]), float(p[1])) for i, p in zip(ids, pos)]


def goal_of(d, ns):
    key = f"{ns}_goal"
    if key in d.files:
        g = d[key]
        if g.size >= 2 and math.isfinite(float(g[0])):
            return float(g[0]), float(g[1])
    return None


def draw_goal(img, goal, info, shape, panel_size):
    if goal is None:
        return
    to_panel = letterbox_mapper(shape, panel_size)
    p = to_panel(world_to_px(goal[0], goal[1], info, shape))
    cv2.drawMarker(img, p, GOAL, cv2.MARKER_DIAMOND, 12, 2, cv2.LINE_AA)


def open_writer(path, fps, size):
    w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), fps, size)
    if not w.isOpened():
        raise RuntimeError(f"could not open H.264 writer for {path}")
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--size", type=int, default=900)
    ap.add_argument("--world", default="office_world")
    args = ap.parse_args()
    truth = {r: truth_markers_in_frame(args.world, r)
             for r in ("leo1", "leo2")}

    snaps = sorted(glob.glob(os.path.join(args.run_dir, "timelapse", "snap*.npz")))
    if not snaps:
        print(f"no snapshots in {args.run_dir}/timelapse", file=sys.stderr)
        return 1

    size = (args.size, int(args.size * 0.75))
    frame_size = (size[0], size[1] + 36)
    # Single-robot runs have no leo2 and no merged map: their grids stay 1x1
    # for the whole run. Rendering them would emit blank videos, so only
    # write the views that actually contain a map.
    live = set()
    for path in (snaps[len(snaps) // 2], snaps[-1]):
        d = np.load(path)
        for name, key in (("leo1", "leo1"), ("leo2", "leo2"),
                          ("merged", "shared")):
            if key in d.files and d[key].size > 1:
                live.add(name)
    writers = {
        name: open_writer(os.path.join(args.run_dir, f"map_explore_{name}.mp4"),
                          args.fps, frame_size)
        for name in ("leo1", "leo2", "merged") if name in live
    }

    t0 = None
    trail1, trail2 = [], []
    for path in snaps:
        d = np.load(path)
        t = float(d["t"])
        t0 = t if t0 is None else t0
        p1, p2 = d["p1"], d["p2"]
        if math.isfinite(float(p1[0])):
            trail1.append((float(p1[0]), float(p1[1])))
        if math.isfinite(float(p2[0])):
            trail2.append((float(p2[0]), float(p2[1])))
        locked = bool(d["locked"])
        tf = d["tf"]

        trail2_shared = []
        if math.isfinite(float(tf[0])):
            c, s = math.cos(float(tf[2])), math.sin(float(tf[2]))
            trail2_shared = [
                (float(tf[0]) + c * x - s * y, float(tf[1]) + s * x + c * y)
                for (x, y) in trail2
            ]

        det1 = detections_of(d, "leo1")
        det2 = detections_of(d, "leo2")
        # On the merged panel leo2's detections are re-expressed in leo1's
        # frame once a transform exists.
        det2_shared = []
        if math.isfinite(float(tf[0])):
            c, s = math.cos(float(tf[2])), math.sin(float(tf[2]))
            det2_shared = [
                (i, float(tf[0]) + c * x - s * y,
                 float(tf[1]) + s * x + c * y) for i, x, y in det2]

        views = {
            "leo1": (d["leo1"], d["leo1_info"], [(trail1, LEO1)], p1, LEO1,
                     "leo1 map", goal_of(d, "leo1"), truth["leo1"], det1),
            "leo2": (d["leo2"], d["leo2_info"], [(trail2, LEO2)], p2, LEO2,
                     "leo2 map", goal_of(d, "leo2"), truth["leo2"], det2),
            "merged": (d["shared"], d["shared_info"],
                       [(trail1, LEO1), (trail2_shared, LEO2)], None, LEO1,
                       "merged map (accepted)", None, truth["leo1"],
                       det1 + det2_shared),
        }
        for name, (grid, info, trail, pose, colour, title, goal,
                   truth_m, det_m) in views.items():
            if name not in writers:
                continue
            img = panel(grid, info, size, trail=trail, pose=pose,
                        colour=colour, title=title)
            if grid is not None and grid.size > 1:
                if goal is not None:
                    draw_goal(img, goal, info, grid.shape, size)
                draw_markers(img, info, grid.shape, size, truth_m, det_m)
            bar = np.full((36, size[0], 3), (250, 250, 250), dtype=np.uint8)
            if name == "merged":
                status = ("aligned" if locked and math.isfinite(float(tf[0]))
                          else "not aligned yet")
            else:
                status = ""
            cv2.putText(bar, f"t = {t - t0:6.0f} s   {status}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1,
                        cv2.LINE_AA)
            cv2.rectangle(bar, (size[0] - 265, 8), (size[0] - 255, 18),
                          TRUTH, 2)
            cv2.putText(bar, "true marker", (size[0] - 248, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1,
                        cv2.LINE_AA)
            cv2.drawMarker(bar, (size[0] - 130, 13), DET,
                           cv2.MARKER_TILTED_CROSS, 10, 2, cv2.LINE_AA)
            cv2.putText(bar, "detected", (size[0] - 115, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1,
                        cv2.LINE_AA)
            writers[name].write(np.vstack([img, bar]))

    for name, w in writers.items():
        w.release()
        p = os.path.join(args.run_dir, f"map_explore_{name}.mp4")
        print(f"{p}: {os.path.getsize(p) / 1e6:.1f} MB, {len(snaps)} frames")
    if not writers:
        print("no non-empty map views to render", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
