#!/usr/bin/env python3
"""Give the BackUp and Spin recoveries enough time to actually finish.

Observation motivating this
---------------------------
Five of fourteen autonomous runs stalled: the rover stopped and never restarted.
`reports/night/n18_ship_s101` shows the recovery layer failing two different
ways, roughly equally often:

    7x  behavior_server: Collision Ahead
    6x  behavior_server: Exceeded time allowance before reaching the
                         DriveOnHeading goal

The second one is arithmetic, not geometry. `BackUp backup_dist="0.25"
backup_speed="0.04"` needs 6.25 s of motion, and `nav2_behaviors` defaults
`time_allowance` to **10 s** — a margin of 1.6x. The collision monitor's
SlowdownZone cuts commands to 75% near an obstacle, which is exactly where a
recovery happens, and 0.25 m at 0.03 m/s is 8.3 s. One 1.5 s velocity-guard
stall on top of that and the backup runs out of time having very nearly
finished. Then the frontier is blacklisted, and enough blacklists end the run.

So: raise the allowance, and raise the speed a little so the manoeuvre is less
marginal to begin with. 0.08 m/s over 0.25 m is 3.1 s of motion inside a 20 s
budget — six times the margin, and still slow enough that the collision monitor
and the guard keep their authority.

`Spin` gets the same treatment: 0.60 rad at the behavior server's
`max_rotational_vel: 0.4` is 1.5 s, but it decelerates into the goal and shares
the same 10 s default.

This does NOT address the `Collision Ahead` half, where the recovery's own
costmap check refuses to move at all. That one is still open.

Usage:
    python3 scripts/apply_recovery_timeout.py [--revert]
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BT = os.path.join(ROOT, 'src', 'leo_nav2_exploration', 'behavior_trees',
                  'navigate_to_pose_doorway_recovery.xml')

ORIGINAL_BACKUP = '<BackUp backup_dist="0.25" backup_speed="0.04"/>'
TUNED_BACKUP = ('<BackUp backup_dist="0.25" backup_speed="0.08" '
                'time_allowance="20"/>')
ORIGINAL_SPIN = '<Spin spin_dist="0.60"/>'
TUNED_SPIN = '<Spin spin_dist="0.60" time_allowance="20"/>'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--revert', action='store_true')
    ap.add_argument('--path', default=BT)
    args = ap.parse_args()

    with open(args.path, encoding='utf-8') as fh:
        text = fh.read()

    pairs = [(ORIGINAL_BACKUP, TUNED_BACKUP), (ORIGINAL_SPIN, TUNED_SPIN)]
    if args.revert:
        pairs = [(b, a) for a, b in pairs]

    changed = 0
    for src, dst in pairs:
        if src in text:
            text = text.replace(src, dst)
            changed += 1
        elif dst in text:
            print(f'  already applied: {dst}')
        else:
            print(f'  NOT FOUND, skipping: {src}', file=sys.stderr)

    if changed:
        with open(args.path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(text)
    print(f'{args.path}: {changed} recovery node(s) '
          f'{"reverted" if args.revert else "given a 20 s allowance"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
