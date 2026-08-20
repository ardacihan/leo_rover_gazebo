#!/usr/bin/env python3
"""Swap SmacPlannerLattice for NavFn in the Nav2 overlay (reversible).

Observation motivating this
---------------------------
`reports/night/n1_ekf_s21` (hybrid + EKF, office_world) covered only 64% of the
world and drove 37.8 m before explore_lite gave up with "All frontiers
traversed/tried out". Its log shows the planner, not the map, running out of
budget:

    8x  GridBased: failed to create plan, no valid path found
    12x controller_server: Failed to make progress
    ~20x Planner loop missed its desired rate of 5.0 Hz (measured 3.9-4.5 Hz)
    8x  Control loop missed its desired rate of 15.0 Hz

`SmacPlannerLattice` searches a state lattice with `max_iterations: 1000000`
and `max_planning_time: 5.0`, and does it while Gazebo is software-rasterising
on the same CPUs. Every second it spends is a second the controller is not
getting its 15 Hz.

NavFn is a Dijkstra grid search: it plans an office-sized costmap in single-
digit milliseconds, and it is the default Nav2 planner precisely because it is
hard to starve. What is given up is kinematic feasibility of the path -- but
the Leo Rover is a skid-steer that turns in place, the controller is
RotationShim + RegulatedPurePursuit, and the shim already rotates the rover
onto the path before following it. A lattice path buys nothing a rover with a
zero turning radius needs.

This also matters off the desk: the rover's own computer is far smaller than
this workstation, and a planner that misses its rate here will miss it worse
there.

Usage:
    python3 scripts/apply_navfn_planner.py --profile sim [--revert]
    python3 scripts/apply_navfn_planner.py --profile real [--revert]
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TARGETS = [
    'src/leo_nav2_exploration/config/{profile}/nav2.yaml',
    'bundle_ref/leo_nav2_exploration/config/{profile}/nav2.yaml',
]

NAVFN_BLOCK = """    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      # Frontier goals sit on the boundary of known space by construction, so
      # a planner that refuses unknown cells cannot reach any of them.
      allow_unknown: true
      # Wider than the lattice's 0.15: explore_lite's goals land wherever the
      # frontier centroid is, which is often a cell or two inside inflation.
      tolerance: 0.25
      use_astar: false
"""

MARKER_BEGIN = '      # >>> navfn-planner (apply_navfn_planner.py)\n'
MARKER_END = '      # <<< navfn-planner\n'


def find_block(text):
    """Return (start, end) of the GridBased: block inside planner_server."""
    m = re.search(r'^    GridBased:\n', text, re.M)
    if not m:
        return None
    start = m.start()
    # The block ends at the next line indented by 4 or fewer spaces that is not
    # blank and not a comment continuation.
    rest = text[m.end():]
    for line_m in re.finditer(r'^(?! {6})(?! *$).*$', rest, re.M):
        return start, m.end() + line_m.start()
    return start, len(text)


def apply(path, revert):
    with open(path, encoding='utf-8') as fh:
        text = fh.read()

    if revert:
        if MARKER_BEGIN not in text:
            print(f'  {path}: no navfn block, nothing to revert')
            return False
        begin = text.index(MARKER_BEGIN)
        end = text.index(MARKER_END) + len(MARKER_END)
        saved = re.search(r'# original:\n((?:      #.*\n)+)', text[begin:end])
        if not saved:
            print(f'  {path}: navfn block has no saved original', file=sys.stderr)
            return False
        original = ''.join(l[8:] if l.startswith('      # ') else l[6:]
                           for l in saved.group(1).splitlines(keepends=True))
        text = text[:begin] + original + text[end:]
        with open(path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(text)
        print(f'  {path}: reverted to SmacPlannerLattice')
        return True

    if MARKER_BEGIN in text:
        print(f'  {path}: already using navfn')
        return False

    span = find_block(text)
    if span is None:
        print(f'  {path}: no GridBased block found', file=sys.stderr)
        return False
    start, end = span
    original = text[start:end]
    if 'NavfnPlanner' in original:
        print(f'  {path}: already NavFn')
        return False

    commented = ''.join('      # ' + l.lstrip('\n') if l.strip() else '      #\n'
                        for l in original.splitlines(keepends=True))
    block = (MARKER_BEGIN + '      # original:\n' + commented + NAVFN_BLOCK
             + MARKER_END)
    text = text[:start] + block + text[end:]
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    print(f'  {path}: SmacPlannerLattice -> NavfnPlanner')
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='sim', choices=('sim', 'real'))
    ap.add_argument('--revert', action='store_true')
    args = ap.parse_args()

    n = 0
    for pattern in TARGETS:
        path = os.path.join(ROOT, pattern.format(profile=args.profile))
        if not os.path.exists(path):
            continue
        n += bool(apply(path, args.revert))
    print(f'{n} file(s) changed')


if __name__ == '__main__':
    main()
