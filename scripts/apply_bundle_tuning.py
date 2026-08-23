#!/usr/bin/env python3
"""Apply (or revert) the tuning changes to the leo_nav2_exploration overlay.

Every change here was motivated by an observed failure in run
``b1_bundle_husarion_office``, where the rover explored cleanly for ~10 minutes
and then parked at (8.88, -5.01) -- a spot with 0.40 m of real clearance --
while the planner logged 116 consecutive "no valid path found" failures against
a goal sitting in 1.05 m of open space.

Applied with ``--profile sim`` and/or ``--profile real``; ``--revert`` restores
the as-shipped values. Both profiles get the same treatment, because the point
of the exercise is a configuration that survives on the physical rover.

Usage:
    python3 apply_bundle_tuning.py --profile sim
    python3 apply_bundle_tuning.py --profile sim --revert
"""

import argparse
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (file, regex, tuned value, as-shipped value, why)
CHANGES = [
    (
        'nav2.yaml', r'(\s*allow_reverse_expansion:\s*)(\S+)', 'true', 'false',
        "The rover cannot plan its way backwards out of anything it drove nose-first "
        "into. This is the single most likely cause of the 116 unplannable goals.",
    ),
    (
        'nav2.yaml', r'(\s*rotation_penalty:\s*)(\S+)', '2.0', '8.0',
        "8.0 heavily penalises in-place rotation, but a skid-steer Leo Rover turns in "
        "place essentially for free. The penalty pushes the lattice planner towards "
        "wide arcs it has no room to execute.",
    ),
    (
        'nav2.yaml', r'(\s*max_planning_time:\s*)(\S+)', '5.0', '2.0',
        "The BT was already logging 'tick rate 100.00 was exceeded', i.e. the stack is "
        "CPU-starved. A 2 s planning budget under that load is a coin flip.",
    ),
    (
        'nav2.yaml', r'(\s*inflation_radius:\s*)(\S+)', '0.35', '0.5',
        "A 0.78 m doorway with 0.5 m of inflation from each jamb is entirely covered by "
        "high cost, so cost_penalty 3.0 makes the only route through it expensive "
        "enough to look unreachable. 0.35 leaves a usable low-cost channel while still "
        "clearing the 0.21 m inscribed radius.",
    ),
    (
        'velocity_guard.yaml', r'(\s*command_timeout:\s*)(\S+)', '1.5', '0.6',
        "velocity_smoother stops publishing between goals, so a 0.6 s timeout made the "
        "guard flap permitted<->command_stale continuously and zero the wheels mid-run.",
    ),
    (
        'nav2.yaml', r'(\s*rotate_to_goal_heading:\s*)(\S+)', 'false', 'true',
        "A frontier goal's heading is meaningless -- the point is to arrive somewhere "
        "new, not to face a particular way. Demanding it made the rover burn whole "
        "minutes spinning on the spot at each goal.",
    ),
    (
        'nav2.yaml', r'(\s*yaw_goal_tolerance:\s*)(\S+)', '3.14', '0.2',
        "Same reason: with 0.2 rad the goal checker refuses to call an otherwise "
        "reached frontier done until the rover has finished a slow pirouette.",
    ),
    (
        'nav2.yaml', r'(\s*slowing_factor:\s*)(\S+)', '1.0', '5.0',
        "RotateToGoalCritic divides its rotation speed by this once inside the xy "
        "tolerance. At 5.0 the observed command was 0.0138 rad/s -- a 90 degree turn "
        "would take two minutes, which is indistinguishable from being stuck.",
    ),
    (
        'frontier.yaml', r'(\s*goal_preemption_min_interval_s:\s*)(\S+)', '15.0', '4.0',
        "Re-dispatching the frontier goal every 4 s cancels Nav2's in-flight recovery "
        "('Failed to get result for backup in node halt!'), so the rover never finishes "
        "the manoeuvre that would free it.",
    ),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['sim', 'real'], required=True)
    ap.add_argument('--revert', action='store_true')
    ap.add_argument('--package-root',
                    default=os.path.join(REPO, 'src', 'leo_nav2_exploration'))
    args = ap.parse_args()

    config_dir = os.path.join(args.package_root, 'config', args.profile)
    if not os.path.isdir(config_dir):
        sys.exit(f'no such config directory: {config_dir}')

    total = 0
    for fname, pattern, tuned, shipped, why in CHANGES:
        path = os.path.join(config_dir, fname)
        if not os.path.isfile(path):
            print(f'  skip {fname}: not present in {args.profile} profile')
            continue
        want = shipped if args.revert else tuned
        text = open(path, encoding='utf-8').read()
        new_text, n = re.subn(pattern, lambda m: m.group(1) + want, text, flags=re.M)
        if n:
            open(path, 'w', encoding='utf-8', newline='\n').write(new_text)
            total += n
            print(f'  {fname}: {pattern.strip()} -> {want}  ({n} site{"s" if n > 1 else ""})')
            if not args.revert:
                print(f'      {why}')
        else:
            print(f'  {fname}: pattern not found: {pattern}')

    verb = 'reverted' if args.revert else 'applied'
    print(f'\n{verb} {total} substitution(s) in {config_dir}')
    print('Rebuild so the installed copy picks it up:')
    print('  colcon build --symlink-install --packages-select leo_nav2_exploration')


if __name__ == '__main__':
    main()
