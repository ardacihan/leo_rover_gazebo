#!/usr/bin/env python3
"""Move the RGBD camera from a VoxelLayer to an ObstacleLayer observation source.

Observed failure (run ``bundlerpp_office_world_realistic``): the local costmap
logged, continuously and forever,

    Sensor origin at (-0.80, 2.35 0.20) is out of map bounds
      (-0.83, 10.00, 0.00) to (3.16, 13.99, 1.18).
      The costmap cannot raytrace for it.

At that moment the live TF ``leo1/odom -> leo1/sensor_camera_link`` was
(1.064, 11.753, 0.200) and the 4x4 m rolling window was correctly centred on
the robot at (1.165, 11.995). The z of the reported origin is right and the xy
is stale, and the 3-D bounds (z 0 -> 1.18 = ``z_voxels`` 24 x 0.05) identify the
reporter as the VoxelLayer rather than the scan ObstacleLayer. Camera latency
was measured at 0.20 s, so this is not staleness of the cloud itself.

Consequence: raytracing is disabled, so nothing is ever cleared from the local
costmap, obstacles accumulate, and the rover ends up boxed in by cells that no
longer correspond to anything real. This is what the planner failures were.

The repository's own ``nav2_params_leo.yaml`` feeds the same
``/leo1/camera/points`` into a plain ObstacleLayer source and shows no such
warning across a full 96%-coverage run, so that wiring is known-good on this
simulator. This script adopts it, and keeps ``clearing: true`` (the repo config
uses ``clearing: False``, which lets depth false-positives accumulate until
they scroll out of the rolling window).

Usage:
    python3 apply_camera_obstacle_layer.py --profile sim [--package-root ...]
"""

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CAMERA_SOURCE = """        camera:
          topic: /leo1/camera/points
          data_type: PointCloud2
          marking: true
          clearing: true
          # Above the floor plane, below the rover's own camera mast, so the
          # layer sees table bases and crossbars the 2-D lidar plane misses.
          min_obstacle_height: 0.06
          max_obstacle_height: 0.60
          obstacle_min_range: 0.25
          obstacle_max_range: 2.5
          raytrace_min_range: 0.25
          raytrace_max_range: 3.0
"""


def strip_voxel_layer(text, camera_topic):
    lines = text.split('\n')
    out, i, removed, rewired = [], 0, False, False
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Drop the voxel_layer entry from any plugins list.
        if stripped == '- voxel_layer':
            removed = True
            i += 1
            continue

        # Add the camera as a second source on the obstacle layer.
        if stripped == 'observation_sources: scan' and not rewired:
            out.append(line.replace('observation_sources: scan',
                                    'observation_sources: scan camera'))
            rewired = True
            i += 1
            continue

        # Excise the whole voxel_layer block, and insert the camera source
        # (which belongs one level deeper, inside obstacle_layer) in its place.
        if stripped == 'voxel_layer:':
            indent = len(line) - len(line.lstrip())
            out.extend(CAMERA_SOURCE.rstrip('\n').split('\n'))
            i += 1
            while i < len(lines):
                nxt = lines[i]
                if nxt.strip() and (len(nxt) - len(nxt.lstrip())) <= indent:
                    break
                i += 1
            continue

        out.append(line)
        i += 1
    return '\n'.join(out), removed, rewired


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['sim', 'real'], required=True)
    ap.add_argument('--package-root',
                    default=os.path.join(REPO, 'src', 'leo_nav2_exploration'))
    args = ap.parse_args()

    path = os.path.join(args.package_root, 'config', args.profile, 'nav2.yaml')
    if not os.path.isfile(path):
        sys.exit(f'no such file: {path}')

    topic = ('/leo1/camera/points' if args.profile == 'sim'
             else '/camera/camera/depth/color/points')
    text = open(path, encoding='utf-8').read()
    if 'voxel_layer' not in text:
        print(f'{path}: already converted')
        return
    new_text, removed, rewired = strip_voxel_layer(text, topic)
    if args.profile == 'real':
        new_text = new_text.replace('topic: /leo1/camera/points', f'topic: {topic}')
    open(path, 'w', encoding='utf-8', newline='\n').write(new_text)
    print(f'{path}: voxel_layer removed={removed} sources_rewired={rewired} topic={topic}')


if __name__ == '__main__':
    main()
