#!/usr/bin/env python3
"""Render the shadow-replay outputs onto the drive bag's timeline.

Consumes the bag recorded by replay_drive_wsl.sh and produces videos on the
same tick grid as extract_bag.py (same t0, same fps), so the dashboard can
scrub every panel with one slider:

    map.mp4             SLAM map growth + robot pose/path in the map frame,
                        current Nav2 plan, its goal, explore_lite frontiers
    global_costmap.mp4  global costmap with the same overlays
    local_costmap.mp4   rolling local costmap, robot-centred, with footprint
    map_final.png       last SLAM map with the full path
    logic.json          goals, Nav2/safety events, driven vs shadow commands

    python3 extract_replay.py <shadow_bag> <orig_bag_metadata.yaml> <outdir>
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BG = (28, 26, 24)
FREE = (52, 50, 48)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(bag):
    return AnyReader([Path(bag)],
                     default_typestore=get_typestore(Stores.ROS2_HUMBLE))


class SE2:
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def compose(self, o):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(self.x + c * o.x - s * o.y,
                   self.y + s * o.x + c * o.y, self.yaw + o.yaw)


def grid_of(msg):
    h, w = msg.info.height, msg.info.width
    return (np.asarray(msg.data, dtype=np.int8).reshape(h, w),
            msg.info.origin.position.x, msg.info.origin.position.y,
            msg.info.resolution)


def render_map(grid, ox, oy, res, canvas):
    """Paint an OccupancyGrid into the fixed canvas (map colors)."""
    img, cx0, cy0, cres = canvas
    px0 = int((ox - cx0) / cres)
    py0 = int((oy - cy0) / cres)
    h, w = grid.shape
    view = img[py0:py0 + h, px0:px0 + w]
    if view.shape[:2] != (h, w):
        h = min(h, view.shape[0])
        w = min(w, view.shape[1])
        grid = grid[:h, :w]
        view = img[py0:py0 + h, px0:px0 + w]
    known = grid >= 0
    view[known] = FREE
    occ = grid >= 65
    view[occ] = (235, 235, 240)
    mid = (grid > 0) & (grid < 65)
    view[mid] = (120, 120, 125)


def render_costmap(grid, ox, oy, res, canvas):
    img, cx0, cy0, cres = canvas
    px0 = int((ox - cx0) / cres)
    py0 = int((oy - cy0) / cres)
    h, w = grid.shape
    view = img[py0:py0 + h, px0:px0 + w]
    if view.shape[:2] != (h, w):
        h = min(h, view.shape[0])
        w = min(w, view.shape[1])
        grid = grid[:h, :w]
        view = img[py0:py0 + h, px0:px0 + w]
    g = grid.astype(np.int16)
    view[g == 0] = FREE
    infl = (g > 0) & (g < 99)
    if infl.any():
        v = (g[infl] * 1.6 + 60).clip(0, 220).astype(np.uint8)
        view[infl] = np.stack([v, (v * 0.45).astype(np.uint8),
                               np.full_like(v, 30)], axis=-1)
    view[g == 99] = (0, 140, 255)
    view[g == 100] = (0, 0, 255)


def colour_local(grid):
    g = grid.astype(np.int16)
    img = np.zeros((*g.shape, 3), np.uint8)
    img[:] = BG
    img[g == 0] = FREE
    infl = (g > 0) & (g < 99)
    if infl.any():
        v = (g[infl] * 1.6 + 60).clip(0, 220).astype(np.uint8)
        img[infl] = np.stack([v, (v * 0.45).astype(np.uint8),
                              np.full_like(v, 30)], axis=-1)
    img[g == 99] = (0, 140, 255)
    img[g == 100] = (0, 0, 255)
    img[g == -1] = (60, 58, 56)
    return img


EVENT_PATTERNS = [
    ('goal', 'bt_navigator', re.compile(r'Begin navigating.*to \(([-\d.]+), ([-\d.]+)\)')),
    ('goal_ok', 'bt_navigator', re.compile(r'Goal succeeded')),
    ('goal_fail', 'bt_navigator', re.compile(r'Goal failed|Aborting handle')),
    ('no_progress', 'controller_server', re.compile(r'Failed to make progress')),
    ('recovery', 'behavior_server', re.compile(r'Running|Collision Ahead')),
    ('stop', 'collision_monitor', re.compile(r'stop|Stop')),
    ('explore', 'explore_node', re.compile(r'Sending goal|Exploration stopped|blacklist')),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('shadow_bag')
    ap.add_argument('orig_meta')
    ap.add_argument('outdir')
    ap.add_argument('--fps', type=float, default=8.0)
    args = ap.parse_args()

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    om = yaml.safe_load(open(args.orig_meta))['rosbag2_bagfile_information']
    t0 = om['starting_time']['nanoseconds_since_epoch']
    dur = om['duration']['nanoseconds'] / 1e9
    nframes = int(dur * args.fps)
    step = 1e9 / args.fps

    # ---- prepass: union bounds of every /map and /global_costmap frame
    x0 = y0 = math.inf
    x1 = y1 = -math.inf
    res = 0.05
    with open_reader(args.shadow_bag) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ('/map', '/global_costmap/costmap')]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            i = msg.info
            res = i.resolution
            x0 = min(x0, i.origin.position.x)
            y0 = min(y0, i.origin.position.y)
            x1 = max(x1, i.origin.position.x + i.width * i.resolution)
            y1 = max(y1, i.origin.position.y + i.height * i.resolution)
    if not math.isfinite(x0):
        sys.exit('shadow bag has no /map or /global_costmap/costmap')
    cw = int(math.ceil((x1 - x0) / res))
    ch = int(math.ceil((y1 - y0) / res))
    print(f'canvas {cw}x{ch} @ {res} m, {nframes} ticks')

    def to_px(x, y):
        return (int((x - x0) / res), int((y - y0) / res))

    scale = max(1, min(3, 900 // max(cw, ch)))
    vsize = (cw * scale, ch * scale)

    def writer(name, size):
        w = cv2.VideoWriter(str(out / name), cv2.VideoWriter_fourcc(*'avc1'),
                            args.fps, size)
        if not w.isOpened():
            sys.exit(f'VideoWriter failed: {name}')
        return w

    vw_map = writer('map.mp4', vsize)
    vw_gc = writer('global_costmap.mp4', vsize)
    LC_PX = 480  # 4 m window at 2.5 cm, x3 -- fixed so every tick has a frame
    vw_lc = writer('local_costmap.mp4', (LC_PX, LC_PX))

    latest = {'map': None, 'gc': None, 'lc': None, 'plan': None,
              'plan_t': -1e9, 'frontiers': [], 'o2b': None, 'm2o': SE2()}
    path_map = []
    events, goals = [], []
    cmd, shadow = [], []
    tick = 0
    next_tick = t0

    def pose_map():
        if latest['o2b'] is None:
            return None
        p = latest['m2o'].compose(latest['o2b'])
        return (p.x, p.y, p.yaw)

    def draw_overlays(img, t):
        pose = pose_map()
        s = scale
        if len(path_map) > 1:
            pts = np.array([[px * s, py * s] for px, py in path_map], np.int32)
            cv2.polylines(img, [pts], False, (90, 190, 90), 1, cv2.LINE_AA)
        for fx, fy in latest['frontiers'][:400]:
            px, py = to_px(fx, fy)
            if 0 <= px < cw and 0 <= py < ch:
                cv2.circle(img, (px * s, py * s), 2, (60, 215, 235), -1)
        if latest['plan'] is not None and t - latest['plan_t'] < 5.0:
            pts = np.array([[px * s, py * s] for px, py in latest['plan']],
                           np.int32)
            if len(pts) > 1:
                cv2.polylines(img, [pts], False, (60, 220, 60), 2, cv2.LINE_AA)
            gx, gy = latest['plan'][-1]
            cv2.drawMarker(img, (gx * s, gy * s), (0, 160, 255),
                           cv2.MARKER_STAR, 12 * s // 2, 2)
        if pose is not None:
            x, y, yaw = pose
            px, py = to_px(x, y)
            r = max(2, int(0.21 / res)) * s
            cv2.circle(img, (px * s, py * s), r, (80, 160, 255), 2, cv2.LINE_AA)
            hx, hy = to_px(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
            cv2.line(img, (px * s, py * s), (hx * s, hy * s),
                     (80, 160, 255), 2, cv2.LINE_AA)
        return img

    def emit(now):
        nonlocal tick, vw_lc
        t = (now - t0) / 1e9
        pose = pose_map()
        if pose is not None:
            px, py = to_px(pose[0], pose[1])
            if not path_map or path_map[-1] != (px, py):
                path_map.append((px, py))

        for src, vw in (('map', vw_map), ('gc', vw_gc)):
            img = np.zeros((ch, cw, 3), np.uint8)
            img[:] = BG
            if latest[src] is not None:
                grid, ox, oy, r = latest[src]
                (render_map if src == 'map' else render_costmap)(
                    grid, ox, oy, r, (img, x0, y0, res))
            if scale > 1:
                img = cv2.resize(img, vsize, interpolation=cv2.INTER_NEAREST)
            img = draw_overlays(img, t)
            vw.write(cv2.flip(img, 0))

        if latest['lc'] is not None:
            grid, ox, oy, r = latest['lc']
            img = colour_local(grid)
            if latest['o2b'] is not None:
                o = latest['o2b']
                cx = int((o.x - ox) / r)
                cy = int((o.y - oy) / r)
                box = np.array([
                    [cx + int((0.21 * 1.414 * math.cos(o.yaw + a)) / r),
                     cy + int((0.21 * 1.414 * math.sin(o.yaw + a)) / r)]
                    for a in (0.785, 2.356, 3.927, 5.498)], np.int32)
                cv2.polylines(img, [box], True, (80, 220, 80), 1)
            img = cv2.flip(cv2.resize(img, (LC_PX, LC_PX),
                                      interpolation=cv2.INTER_NEAREST), 0)
        else:
            img = np.zeros((LC_PX, LC_PX, 3), np.uint8)
            img[:] = BG
        vw_lc.write(img)
        tick += 1
        if tick % 600 == 0:
            print(f'  tick {tick}/{nframes}', flush=True)

    with open_reader(args.shadow_bag) as reader:
        want = ['/map', '/global_costmap/costmap', '/local_costmap/costmap',
                '/plan', '/explore/frontiers', '/tf', '/cmd_vel',
                '/cmd_vel_shadow', '/rosout']
        conns = [c for c in reader.connections if c.topic in want]
        for conn, ts, raw in reader.messages(connections=conns):
            while ts >= next_tick and tick < nframes:
                emit(next_tick)
                next_tick += step
            msg = reader.deserialize(raw, conn.msgtype)
            t = (ts - t0) / 1e9
            top = conn.topic
            if top == '/tf':
                for tr in msg.transforms:
                    tt = tr.transform.translation
                    se2 = SE2(tt.x, tt.y, yaw_of(tr.transform.rotation))
                    if tr.child_frame_id == 'base_footprint':
                        latest['o2b'] = se2
                    elif tr.child_frame_id == 'odom':
                        latest['m2o'] = se2
            elif top == '/map':
                latest['map'] = grid_of(msg)
            elif top == '/global_costmap/costmap':
                latest['gc'] = grid_of(msg)
            elif top == '/local_costmap/costmap':
                latest['lc'] = grid_of(msg)
            elif top == '/plan':
                pts = [to_px(p.pose.position.x, p.pose.position.y)
                       for p in msg.poses[::3]]
                if pts:
                    latest['plan'] = pts
                    latest['plan_t'] = t
            elif top == '/explore/frontiers':
                pts = []
                for m in msg.markers:
                    for p in m.points:
                        pts.append((p.x, p.y))
                latest['frontiers'] = pts
            elif top == '/cmd_vel':
                cmd.append([round(t, 2), round(msg.linear.x, 3),
                            round(msg.angular.z, 3)])
            elif top == '/cmd_vel_shadow':
                shadow.append([round(t, 2), round(msg.linear.x, 3),
                               round(msg.angular.z, 3)])
            elif top == '/rosout':
                name = msg.name.split('.')[-1]
                text = msg.msg
                for kind, node, pat in EVENT_PATTERNS:
                    if node in msg.name:
                        m = pat.search(text)
                        if m:
                            ev = {'t': round(t, 1), 'kind': kind,
                                  'node': name, 'msg': text[:160]}
                            events.append(ev)
                            if kind == 'goal':
                                goals.append({'t': round(t, 1),
                                              'x': float(m.group(1)),
                                              'y': float(m.group(2))})
                            break
        while tick < nframes:
            emit(next_tick)
            next_tick += step

    for vw in (vw_map, vw_gc, vw_lc):
        if vw is not None:
            vw.release()

    img = np.zeros((ch, cw, 3), np.uint8)
    img[:] = BG
    if latest['map'] is not None:
        grid, ox, oy, r = latest['map']
        render_map(grid, ox, oy, r, (img, x0, y0, res))
    if scale > 1:
        img = cv2.resize(img, vsize, interpolation=cv2.INTER_NEAREST)
    img = draw_overlays(img, dur)
    cv2.imwrite(str(out / 'map_final.png'), cv2.flip(img, 0))

    (out / 'logic.json').write_text(json.dumps({
        'meta': {'t0_ns': t0, 'duration_s': round(dur, 1), 'fps': args.fps,
                 'frames': tick, 'canvas': {'x0': x0, 'y0': y0, 'res': res,
                                            'w': cw, 'h': ch, 'scale': scale}},
        'goals': goals,
        'events': events[:2000],
        'cmd': cmd,
        'shadow': shadow,
    }))
    print(f'done: {tick} ticks, {len(goals)} goals, {len(events)} events')


if __name__ == '__main__':
    main()
