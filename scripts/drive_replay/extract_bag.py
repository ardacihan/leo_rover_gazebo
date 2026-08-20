#!/usr/bin/env python3
"""Extract a synced dashboard media set from a real-rover drive bag.

One streaming pass over the bag renders three H.264 videos on a shared
timeline (default 8 fps), so a dashboard can scrub all panels with one
slider:

    color.mp4       RealSense colour stream (nearest frame per tick)
    depth.mp4       depth stream, colormapped 0..5 m
    lidar.mp4       odom-frame view: accumulated scan hits, live scan,
                    path so far, rover footprint + heading
    lidar_map.png   final accumulated lidar map with the full path
    data.json       trajectory, cmd_vel, battery, per-tick source stamps

    python3 extract_bag.py <bagdir> <outdir> [--fps 8] [--max-range 12]

Poses come from /merged_odom (the rover's wheel+IMU fusion, identical to the
bag's odom->base_footprint TF); the scan is placed with the static
base_footprint->laser_frame transform from /tf_static.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(bag):
    return AnyReader([Path(bag)],
                     default_typestore=get_typestore(Stores.ROS2_HUMBLE))


def prepass(bag):
    """Trajectory bounds and static laser transform, from the small topics."""
    xs, ys = [], []
    laser = (0.077, 0.040, 0.0)  # fallback: measured mount, no rotation
    with open_reader(bag) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ('/merged_odom', '/tf_static')]
        chain = {}
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/merged_odom':
                p = msg.pose.pose.position
                if math.isfinite(p.x) and math.isfinite(p.y):
                    xs.append(p.x)
                    ys.append(p.y)
            else:
                for tr in msg.transforms:
                    chain[(tr.header.frame_id, tr.child_frame_id)] = tr.transform
        bl = chain.get(('base_footprint', 'base_link'))
        lf = chain.get(('base_link', 'laser_frame'))
        if lf is not None:
            t = lf.translation
            yaw = yaw_of(lf.rotation)
            if bl is not None:
                yaw += yaw_of(bl.rotation)
            laser = (t.x, t.y, yaw)
    if not xs:
        sys.exit('no /merged_odom in bag')
    return (min(xs), max(xs), min(ys), max(ys)), laser


class LidarCanvas:
    """Odom-frame raster: accumulated hits, path, live scan, rover marker."""

    def __init__(self, bounds, max_range, res=0.05, max_px=920):
        margin = min(max_range, 6.0) + 1.0
        x0, x1, y0, y1 = (bounds[0] - margin, bounds[1] + margin,
                          bounds[2] - margin, bounds[3] + margin)
        w, h = x1 - x0, y1 - y0
        self.res = max(res, w / max_px, h / max_px)
        self.x0, self.y0 = x0, y0
        self.w = int(math.ceil(w / self.res))
        self.h = int(math.ceil(h / self.res))
        self.hits = np.zeros((self.h, self.w), dtype=np.uint16)
        self.path_px = []

    def to_px(self, x, y):
        return (int((x - self.x0) / self.res), int((y - self.y0) / self.res))

    def add_scan(self, pts):
        for x, y in pts:
            px, py = self.to_px(x, y)
            if 0 <= px < self.w and 0 <= py < self.h:
                self.hits[py, px] = min(self.hits[py, px] + 1, 60000)

    def add_pose(self, x, y):
        self.path_px.append(self.to_px(x, y))

    def render(self, scan_pts, pose):
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        img[:] = (28, 26, 24)
        seen = self.hits > 0
        solid = self.hits >= 3
        img[seen] = (95, 95, 100)
        img[solid] = (235, 235, 240)
        if len(self.path_px) > 1:
            cv2.polylines(img, [np.array(self.path_px, np.int32)], False,
                          (90, 190, 90), 1, cv2.LINE_AA)
        for x, y in scan_pts:
            px, py = self.to_px(x, y)
            if 0 <= px < self.w and 0 <= py < self.h:
                img[py, px] = (60, 210, 255)
        if pose is not None:
            x, y, yaw = pose
            cx, cy = self.to_px(x, y)
            r = max(2, int(0.21 / self.res))
            cv2.circle(img, (cx, cy), r, (80, 160, 255), 1, cv2.LINE_AA)
            hx, hy = self.to_px(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
            cv2.line(img, (cx, cy), (hx, hy), (80, 160, 255), 2, cv2.LINE_AA)
        return cv2.flip(img, 0)  # +y up


def scan_points(scan, pose, laser, max_range):
    x, y, yaw = pose
    lx, ly, lyaw = laser
    ox = x + lx * math.cos(yaw) - ly * math.sin(yaw)
    oy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
    a0 = yaw + lyaw
    rng = np.asarray(scan.ranges, dtype=np.float32)
    ang = scan.angle_min + np.arange(len(rng), dtype=np.float32) * scan.angle_increment
    ok = np.isfinite(rng) & (rng > max(scan.range_min, 0.05)) & (rng <= max_range)
    rng, ang = rng[ok], ang[ok] + a0
    pts = np.stack([ox + rng * np.cos(ang), oy + rng * np.sin(ang)], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def writer(path, size, fps):
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'avc1'), fps, size)
    if not w.isOpened():
        sys.exit(f'VideoWriter failed for {path}')
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('outdir')
    ap.add_argument('--fps', type=float, default=8.0)
    ap.add_argument('--max-range', type=float, default=12.0)
    ap.add_argument('--color-width', type=int, default=640)
    args = ap.parse_args()

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    bounds, laser = prepass(args.bag)
    canvas = LidarCanvas(bounds, args.max_range)
    print(f'bounds {bounds}, laser mount {laser}, '
          f'canvas {canvas.w}x{canvas.h} @ {canvas.res:.3f} m')

    color_size = None
    depth_size = None
    vw_color = vw_depth = None
    vw_lidar = writer(out / 'lidar.mp4', (canvas.w, canvas.h), args.fps)

    latest = {'color': None, 'depth': None, 'scan': None, 'pose': None,
              'speed': 0.0, 'wz': 0.0}
    stamps = {'color': None, 'depth': None, 'scan': None}
    traj, cmds, battery, tick_rows = [], [], [], []
    dist = 0.0
    last_xy = None

    with open_reader(args.bag) as reader:
        topics = ['/bag/color/compressed', '/bag/depth/compressed', '/scan',
                  '/merged_odom', '/cmd_vel', '/rob_2/firmware/battery_averaged']
        conns = [c for c in reader.connections if c.topic in topics]
        t0 = reader.start_time
        dur = (reader.end_time - t0) / 1e9
        nframes = int(dur * args.fps)
        step = 1e9 / args.fps
        next_tick = t0
        tick = 0

        def emit(now):
            nonlocal tick
            t = (now - t0) / 1e9
            pose = latest['pose']
            # colour
            if vw_color is not None:
                frame = latest['color'] if latest['color'] is not None else \
                    np.zeros((color_size[1], color_size[0], 3), np.uint8)
                vw_color.write(frame)
            # depth
            if vw_depth is not None:
                frame = latest['depth'] if latest['depth'] is not None else \
                    np.zeros((depth_size[1], depth_size[0], 3), np.uint8)
                vw_depth.write(frame)
            # lidar
            pts = latest['scan'] if latest['scan'] is not None else []
            vw_lidar.write(canvas.render(pts, pose))
            if pose is not None:
                traj.append([round(t, 2), round(pose[0], 3), round(pose[1], 3),
                             round(pose[2], 3), round(latest['speed'], 3)])
            tick_rows.append([round(t, 2),
                              stamps['color'], stamps['depth'], stamps['scan']])
            tick += 1
            if tick % 400 == 0:
                print(f'  tick {tick}/{nframes} t={t:.0f}s', flush=True)

        for conn, ts, raw in reader.messages(connections=conns):
            while ts >= next_tick and tick < nframes:
                emit(next_tick)
                next_tick += step
            msg = reader.deserialize(raw, conn.msgtype)
            t = (ts - t0) / 1e9
            if conn.topic == '/bag/color/compressed':
                img = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is None:
                    continue
                if color_size is None:
                    scale = args.color_width / img.shape[1]
                    color_size = (args.color_width, int(img.shape[0] * scale))
                    vw_color = writer(out / 'color.mp4', color_size, args.fps)
                latest['color'] = cv2.resize(img, color_size)
                stamps['color'] = round(t, 2)
            elif conn.topic == '/bag/depth/compressed':
                d = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                 cv2.IMREAD_UNCHANGED)
                if d is None:
                    continue
                mm = np.clip(d.astype(np.float32), 0, 5000) / 5000.0
                vis = cv2.applyColorMap((mm * 255).astype(np.uint8),
                                        cv2.COLORMAP_TURBO)
                vis[d == 0] = (0, 0, 0)
                if depth_size is None:
                    depth_size = (424, int(424 * d.shape[0] / d.shape[1]))
                    vw_depth = writer(out / 'depth.mp4', depth_size, args.fps)
                latest['depth'] = cv2.resize(vis, depth_size)
                stamps['depth'] = round(t, 2)
            elif conn.topic == '/scan':
                if latest['pose'] is not None:
                    pts = scan_points(msg, latest['pose'], laser, args.max_range)
                    latest['scan'] = pts
                    canvas.add_scan(pts)
                    stamps['scan'] = round(t, 2)
            elif conn.topic == '/merged_odom':
                p = msg.pose.pose.position
                if not (math.isfinite(p.x) and math.isfinite(p.y)):
                    continue
                latest['pose'] = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
                v = msg.twist.twist
                latest['speed'] = math.hypot(v.linear.x, v.linear.y)
                latest['wz'] = v.angular.z
                canvas.add_pose(p.x, p.y)
                if last_xy is not None:
                    dist += math.hypot(p.x - last_xy[0], p.y - last_xy[1])
                last_xy = (p.x, p.y)
            elif conn.topic == '/cmd_vel':
                cmds.append([round(t, 2), round(msg.linear.x, 3),
                             round(msg.angular.z, 3)])
            elif conn.topic == '/rob_2/firmware/battery_averaged':
                if not battery or t - battery[-1][0] > 5.0:
                    battery.append([round(t, 2), round(float(msg.data), 2)])
        while tick < nframes:
            emit(next_tick)
            next_tick += step

    for w in (vw_color, vw_depth, vw_lidar):
        if w is not None:
            w.release()

    final = canvas.render([], latest['pose'])
    cv2.imwrite(str(out / 'lidar_map.png'), final)

    meta = {
        'bag': str(args.bag), 't0_ns': t0,
        'duration_s': round(dur, 1), 'fps': args.fps,
        'frames': tick, 'distance_m': round(dist, 1),
        'canvas': {'x0': canvas.x0, 'y0': canvas.y0, 'res': canvas.res,
                   'w': canvas.w, 'h': canvas.h},
        'laser_mount': laser, 'max_range': args.max_range,
    }
    (out / 'data.json').write_text(json.dumps(
        {'meta': meta, 'traj': traj, 'cmd': cmds, 'battery': battery}))
    print(f'done: {tick} frames, {dist:.1f} m driven, -> {out}')


if __name__ == '__main__':
    main()
