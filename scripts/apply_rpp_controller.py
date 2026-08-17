#!/usr/bin/env python3
"""Swap the overlay's DWB controller for Regulated Pure Pursuit, and open up the
speed envelope so a time-boxed exploration run can actually cover ground.

Motivation, all measured rather than assumed:

DWB never commanded forward motion in either bundle run. On office_world the
final command was ``linear.x = 0.0, angular.z = 0.0138`` while the rover sat in
0.632 m of open space -- and 0.0138 is exactly sample index 15 of
``vtheta_samples: 30`` over +/-0.4, i.e. the sample nearest zero. DWB was
scoring every genuine trajectory as worse than standing still. With
``ObstacleFootprint`` at ``scale: 0.03`` and four path/goal critics at 20-28,
that critic balance is doing the damage, and re-tuning seven interacting critic
weights blind is not a good use of a night.

Regulated Pure Pursuit is Nav2's recommended controller for exactly this case
-- a differential-drive robot in tight indoor space. It has a handful of
physically meaningful parameters instead of a critic balance, slows itself
near obstacles and on curves by construction, and is kept behind the same
RotationShimController so in-place turns still happen first.

Usage:
    python3 apply_rpp_controller.py --profile sim [--package-root ...]
"""

import argparse
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Leo Rover does ~0.4 m/s; 0.26 leaves margin while roughly doubling the
# ground covered inside a fixed wall-clock cap. The as-shipped 0.12 m/s meant
# a single 7.8 m frontier goal needed 65 s of uninterrupted driving.
MAX_LIN = '0.26'
MAX_ANG = '0.9'

RPP_BLOCK = """    FollowPath:
      plugin: nav2_rotation_shim_controller::RotationShimController
      primary_controller: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
      angular_dist_threshold: 0.5
      angular_disengage_threshold: 0.25
      forward_sampling_distance: 0.35
      rotate_to_heading_angular_vel: 0.6
      max_angular_accel: 1.2
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: false
      closed_loop: true
      desired_linear_vel: %(lin)s
      lookahead_dist: 0.5
      min_lookahead_dist: 0.25
      max_lookahead_dist: 0.8
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: true
      transform_tolerance: 0.3
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.5
      # Slow down for tight curves and near obstacles instead of refusing to move.
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.5
      regulated_linear_scaling_min_speed: 0.10
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.35
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 4.0
      # In-place rotation is cheap on a skid-steer chassis; let it use that
      # rather than needing room for an arc.
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.5
      rotate_to_heading_angular_vel: 0.6
      allow_reversing: false
      max_robot_pose_search_dist: 5.0
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.5
""" % {'lin': MAX_LIN}


def replace_followpath(text):
    lines = text.split('\n')
    start = next((i for i, l in enumerate(lines) if l.strip() == 'FollowPath:'), None)
    if start is None:
        return text, 0
    # The block ends at the next line indented 4 spaces or less that is not blank.
    end = len(lines)
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        if indent <= 4:
            end = i
            break
    return '\n'.join(lines[:start] + RPP_BLOCK.rstrip('\n').split('\n') + lines[end:]), 1


SCALARS = [
    ('nav2.yaml', r'(\s*max_vel_x:\s*)(\S+)', MAX_LIN),
    ('nav2.yaml', r'(\s*max_speed_xy:\s*)(\S+)', MAX_LIN),
    ('nav2.yaml', r'(\s*max_vel_theta:\s*)(\S+)', MAX_ANG),
    ('velocity_guard.yaml', r'(\s*max_linear_speed:\s*)(\S+)', MAX_LIN),
    ('velocity_guard.yaml', r'(\s*max_angular_speed:\s*)(\S+)', MAX_ANG),
    # A 7.8 m goal at 0.26 m/s still needs ~30 s of clear driving; preempting
    # every 15 s restarted navigation from the same spot forever.
    ('frontier.yaml', r'(\s*goal_preemption_min_interval_s:\s*)(\S+)', '90.0'),
    # The 0.34 m slowdown box was permanently triggered, holding the rover at
    # 45% speed for the whole run. 0.28 m still clears the 0.22 m half-width.
    ('collision_monitor.yaml', r'(\s*slowdown_ratio:\s*)(\S+)', '0.75'),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['sim', 'real'], required=True)
    ap.add_argument('--package-root',
                    default=os.path.join(REPO, 'src', 'leo_nav2_exploration'))
    args = ap.parse_args()

    config_dir = os.path.join(args.package_root, 'config', args.profile)
    if not os.path.isdir(config_dir):
        sys.exit(f'no such config directory: {config_dir}')

    nav2 = os.path.join(config_dir, 'nav2.yaml')
    text = open(nav2, encoding='utf-8').read()
    text, n = replace_followpath(text)
    open(nav2, 'w', encoding='utf-8', newline='\n').write(text)
    print(f'  nav2.yaml: FollowPath -> RegulatedPurePursuit ({n} block)')

    for fname, pattern, value in SCALARS:
        path = os.path.join(config_dir, fname)
        if not os.path.isfile(path):
            print(f'  skip {fname}: not present')
            continue
        body = open(path, encoding='utf-8').read()
        body, k = re.subn(pattern, lambda m: m.group(1) + value, body, flags=re.M)
        if k:
            open(path, 'w', encoding='utf-8', newline='\n').write(body)
        print(f'  {fname}: {pattern.strip()} -> {value} ({k})')

    # The smoother's limit vector has to rise with the controller's, or it
    # silently clamps everything back down to the old envelope.
    body = open(nav2, encoding='utf-8').read()
    body = body.replace(
        'max_velocity:\n    - 0.12\n    - 0.0\n    - 0.4',
        f'max_velocity:\n    - {MAX_LIN}\n    - 0.0\n    - {MAX_ANG}')
    body = body.replace(
        'min_velocity:\n    - -0.05\n    - 0.0\n    - -0.4',
        f'min_velocity:\n    - -0.10\n    - 0.0\n    - -{MAX_ANG}')
    open(nav2, 'w', encoding='utf-8', newline='\n').write(body)
    print('  nav2.yaml: velocity_smoother envelope raised to match')

    print(f'\ndone for {config_dir}')


if __name__ == '__main__':
    main()
