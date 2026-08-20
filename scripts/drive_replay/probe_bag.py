#!/usr/bin/env python3
"""Print the facts needed to build the extractor: frames, formats, rates."""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag = Path(sys.argv[1])
with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
    conns = {c.topic: c for c in reader.connections}
    start, end = reader.start_time, reader.end_time
    print(f'duration {(end-start)/1e9:.1f}s')

    def first(topic, n=1):
        out = []
        for conn, ts, raw in reader.messages(connections=[conns[topic]]):
            out.append((ts, reader.deserialize(raw, conn.msgtype)))
            if len(out) >= n:
                break
        return out

    for ts, msg in first('/tf_static', 6):
        for tr in msg.transforms:
            t = tr.transform.translation
            print(f'static {tr.header.frame_id} -> {tr.child_frame_id} '
                  f'({t.x:.3f},{t.y:.3f},{t.z:.3f})')

    ts, msg = first('/tf')[0]
    for tr in msg.transforms:
        print(f'tf {tr.header.frame_id} -> {tr.child_frame_id}')

    ts, scan = first('/scan')[0]
    print(f'scan frame={scan.header.frame_id} n={len(scan.ranges)} '
          f'range=[{scan.range_min},{scan.range_max}] '
          f'angle=[{scan.angle_min:.2f},{scan.angle_max:.2f}]')

    ts, img = first('/bag/color/compressed')[0]
    print(f'color format={img.format!r} bytes={len(img.data)} frame={img.header.frame_id}')
    ts, img = first('/bag/depth/compressed')[0]
    print(f'depth format={img.format!r} bytes={len(img.data)} frame={img.header.frame_id} head={bytes(img.data[:24])!r}')

    ts, ci = first('/rob_4/camera/depth/camera_info')[0]
    print(f'depth caminfo frame={ci.header.frame_id} {ci.width}x{ci.height} k={list(ci.k)}')
    ts, ci = first('/rob_4/camera/color/camera_info')[0]
    print(f'color caminfo frame={ci.header.frame_id} {ci.width}x{ci.height}')

    ts, od = first('/merged_odom')[0]
    print(f'merged_odom frame={od.header.frame_id} child={od.child_frame_id}')
    ts, od = first('/wheel_odom')[0]
    print(f'wheel_odom frame={od.header.frame_id} child={od.child_frame_id}')
