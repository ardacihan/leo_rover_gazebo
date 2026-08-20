#!/usr/bin/env python3
"""What does the depth camera actually put into the costmap band?

For a given bag time: decode the nearest depth frame, project every pixel,
transform into base_footprint using the bag's static TF chain, and report
where the points land relative to the ObstacleLayer's 0.06-0.60 m band --
separating solid returns from flying pixels (points on a depth edge whose
value differs sharply from the local median).

    python3 analyze_depth.py <bag> <t_seconds> [...]
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    bag = Path(sys.argv[1])
    times = [float(a) for a in sys.argv[2:]]
    ts_store = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=ts_store) as reader:
        conns = {c.topic: c for c in reader.connections}
        t0 = reader.start_time

        chain = {}
        for conn, ts, raw in reader.messages(connections=[conns['/tf_static']]):
            msg = reader.deserialize(raw, conn.msgtype)
            for tr in msg.transforms:
                t = tr.transform.translation
                chain[(tr.header.frame_id, tr.child_frame_id)] = (
                    np.array([t.x, t.y, t.z]), quat_mat(tr.transform.rotation))

        def compose(*links):
            R = np.eye(3)
            p = np.zeros(3)
            for a, b in links:
                t, Rl = chain[(a, b)]
                p = p + R @ t
                R = R @ Rl
            return p, R

        p_bc, R_bc = compose(('base_footprint', 'base_link'),
                             ('base_link', 'camera_link'),
                             ('camera_link', 'camera_depth_frame'),
                             ('camera_depth_frame', 'camera_depth_optical_frame'))
        print(f'base_footprint -> depth optical: t={np.round(p_bc,3)}')
        pitch = math.degrees(math.asin(-R_bc[2, 0]))

        ci = None
        for conn, ts, raw in reader.messages(
                connections=[conns['/rob_4/camera/depth/camera_info']]):
            ci = reader.deserialize(raw, conn.msgtype)
            break
        fx, fy, cx, cy = ci.k[0], ci.k[4], ci.k[2], ci.k[5]

        want = sorted(times)
        for conn, ts, raw in reader.messages(
                connections=[conns['/bag/depth/compressed']]):
            t = (ts - t0) / 1e9
            if not want or t < want[0]:
                continue
            want.pop(0)
            msg = reader.deserialize(raw, conn.msgtype)
            d = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_UNCHANGED)
            z = d.astype(np.float32) * 1e-3
            med = cv2.medianBlur(d, 5).astype(np.float32) * 1e-3
            flying = np.abs(z - med) > 0.3

            h, w = z.shape
            us, vs = np.meshgrid(np.arange(w), np.arange(h))
            X = (us - cx) / fx * z
            Y = (vs - cy) / fy * z
            ok = (z > 0.2) & (z < 3.2)
            pts = np.stack([X[ok], Y[ok], z[ok]], 1)
            fly = flying[ok]
            world = pts @ R_bc.T + p_bc
            zz = world[:, 2]
            rng = np.hypot(world[:, 0], world[:, 1])
            band = (zz > 0.06) & (zz < 0.60)
            floorish = np.abs(zz) < 0.04
            print(f'\n t={t:.1f}s  {ok.sum()} pts in 0.2-3.2m '
                  f'(camera pitch {pitch:+.1f} deg)')
            print(f'   in costmap band 0.06-0.60m: {band.sum()} '
                  f'({100*band.sum()/len(zz):.1f}%)   flying-pixel share of '
                  f'band: {100*(band&fly).sum()/max(band.sum(),1):.1f}%')
            print(f'   floor +/-4cm: {100*floorish.sum()/len(zz):.1f}%   '
                  f'band pts beyond 2 m: {(band&(rng>2)).sum()} '
                  f'(flying {100*((band&fly)&(rng>2)).sum()/max((band&(rng>2)).sum(),1):.0f}%)')
            for lo, hi in ((0.06, 0.12), (0.12, 0.25), (0.25, 0.45), (0.45, 0.60)):
                m = (zz > lo) & (zz < hi)
                print(f'   z {lo:.2f}-{hi:.2f}: {m.sum():6d} pts, '
                      f'flying {100*(m&fly).sum()/max(m.sum(),1):4.1f}%')
            if not want:
                break


if __name__ == '__main__':
    main()
