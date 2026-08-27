#!/usr/bin/env python3
"""Check a recorded bag actually contains what the run needed.

    python3 tools/verify_bag.py <bag-dir>

Prints every topic with its message count, mean rate and share of the bytes,
then judges the ones that matter against a minimum rate. Reads the .db3 files
directly with sqlite3 — no ROS, no rosbags package, so it runs on the Jetson
right after the recording and on the laptop afterwards.

Run it before you tear the stack down. A bag missing its lidar is recoverable
in the ten minutes you are still in the lab and unrecoverable the moment you
leave, and nothing in `ros2 bag record`'s output tells you: a topic that never
delivered a message is recorded as a topic with zero messages, silently.
"""

import glob
import os
import sqlite3
import sys

# topic -> (minimum mean Hz, why it matters). Anything not listed is reported
# but not judged.
EXPECT = {
    '/scan':                      (4.0,  'lidar; no scan means no map at all'),
    '/tf':                        (5.0,  'every pose in the bag depends on it'),
    '/tf_static':                 (0.0,  'sensor mounts (a handful of msgs)'),
    '/bag/color/compressed':      (1.0,  'camera'),
    '/bag/color/camera_info':     (0.2,  'K and D; without these no offline ArUco'),
    '/wheel_odom':                (5.0,  'odometry'),
    '/rob_2/firmware/imu':        (20.0, 'gyro'),
    '/rob_2/firmware/battery_averaged': (2.0, 'battery, and the liveness canary'),
    '/bag/depth/compressed':      (1.0,  'depth; needed for the camera costmap layer'),
    '/map':                       (0.02, 'SLAM output'),
}


def main():
    if len(sys.argv) < 2:
        sys.exit('usage: verify_bag.py <bag-dir>')
    d = sys.argv[1]
    dbs = sorted(glob.glob(os.path.join(d, '*.db3')))
    if not dbs:
        sys.exit(f'no .db3 files in {d}')
    if not os.path.exists(os.path.join(d, 'metadata.yaml')):
        print('WARNING: no metadata.yaml — the recorder was killed rather than\n'
              '         Ctrl+C\'d. `ros2 bag play` will refuse this bag.\n')

    counts, byts, tmin, tmax = {}, {}, None, None
    for db in dbs:
        con = sqlite3.connect(db)
        for name, n, b, lo, hi in con.execute(
                'select t.name, count(*), sum(length(m.data)), '
                'min(m.timestamp), max(m.timestamp) '
                'from messages m join topics t on t.id = m.topic_id '
                'group by t.name'):
            counts[name] = counts.get(name, 0) + n
            byts[name] = byts.get(name, 0) + (b or 0)
            tmin = lo if tmin is None else min(tmin, lo)
            tmax = hi if tmax is None else max(tmax, hi)
        con.close()

    if tmin is None:
        sys.exit('bag contains no messages at all')
    dur = (tmax - tmin) / 1e9
    total = sum(byts.values())
    size = sum(os.path.getsize(f) for f in dbs)
    print(f'{d}\n  {dur / 60:.1f} min, {len(dbs)} file(s), '
          f'{size / 1048576:.0f} MB on disk, {size / 1048576 / (dur / 60):.0f} MB/min\n')

    print(f'  {"topic":<44}{"msgs":>8}{"Hz":>8}{"MB":>8}{"%":>6}')
    for name in sorted(byts, key=lambda k: -byts[k]):
        n, b = counts[name], byts[name]
        print(f'  {name:<44}{n:>8}{n / dur:>8.1f}{b / 1048576:>8.1f}'
              f'{b / total * 100:>6.1f}')

    problems = []
    for topic, (min_hz, why) in EXPECT.items():
        n = counts.get(topic, 0)
        if n == 0:
            problems.append(f'  ABSENT   {topic}  -- {why}')
        elif min_hz > 0 and n / dur < min_hz:
            problems.append(
                f'  LOW      {topic}  {n / dur:.1f} Hz < {min_hz} Hz  -- {why}')

    print()
    if not problems:
        print('  OK — everything expected is present at a sane rate.')
        return
    print('  PROBLEMS:')
    print('\n'.join(problems))
    print('\n  A topic can be ABSENT because it was never recorded (check the\n'
          '  recorder\'s "note: ... absent, skipping" lines) or because it was\n'
          '  recorded and never published. tools/preflight_topics.sh separates\n'
          '  those two, and is worth running before the next leg.')
    sys.exit(1)


if __name__ == '__main__':
    main()
