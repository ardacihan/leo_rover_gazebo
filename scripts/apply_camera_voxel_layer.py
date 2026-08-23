#!/usr/bin/env python3
"""Put the RGBD camera back on a VoxelLayer instead of an ObstacleLayer source.

Why revisit this. The camera was moved to a 2-D ObstacleLayer because the
VoxelLayer reported a frozen sensor origin and disabled raytracing:

    Sensor origin at (-0.80, 2.35 0.20) is out of map bounds
      (-0.83, 10.00, 0.00) to (3.16, 13.99, 1.18)

That was a workaround, not a diagnosis, and VoxelLayer is the better tool for a
depth camera: it accumulates and clears occupancy in 3-D, whereas ObstacleLayer
flattens the cloud to 2-D, under-clears overhangs, and is more exposed to
persistent false marks from depth noise.

The hypothesis worth testing: the local costmap rolls in the **odom** frame, and
when that failure was observed the wheel odometry had drifted about 11 m, so the
rolling window sat at odom y ~ 12 while the reported observation origin sat at
y ~ 2.35 -- roughly one drift-length behind. With the EKF halving odometry
drift, the condition may not arise at all.

Reverses `apply_camera_obstacle_layer.py`: drops the camera source from the
obstacle layer and reinstates the voxel layer, keeping the tuned
`min_obstacle_height` rather than the bundle's original 0.05.

Usage:
    python3 apply_camera_voxel_layer.py --profile sim [--package-root ...]
"""

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

VOXEL_BLOCK = """      voxel_layer:
        plugin: nav2_costmap_2d::VoxelLayer
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 24
        unknown_threshold: 15
        mark_threshold: 0
        max_obstacle_height: 1.2
        footprint_clearing_enabled: true
        combination_method: 1
        observation_sources: depth
        depth:
          topic: {topic}
          data_type: PointCloud2
          marking: true
          clearing: true
          # 0.06 rather than the bundle's 0.05: on real depth the floor plane
          # is noisy, and marking the floor is the classic RealSense failure.
          min_obstacle_height: 0.06
          max_obstacle_height: 0.60
          obstacle_min_range: 0.25
          obstacle_max_range: 2.5
          raytrace_min_range: 0.25
          raytrace_max_range: 3.0
          observation_persistence: 0.0
"""


def convert(text, topic):
    lines = text.split('\n')
    out, i = [], 0
    in_local = False
    removed_camera = rewired = added = False
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if stripped == 'local_costmap:':
            in_local = True
        elif stripped == 'global_costmap:':
            in_local = False

        # Only touch the local costmap; the global one never had a voxel layer.
        if in_local and stripped == '- obstacle_layer' and not added:
            out.append(line)
            out.append(line.replace('- obstacle_layer', '- voxel_layer'))
            added = True
            i += 1
            continue

        if in_local and stripped == 'observation_sources: scan camera':
            out.append(line.replace('scan camera', 'scan'))
            rewired = True
            i += 1
            continue

        # Excise the camera source from the obstacle layer and drop the voxel
        # layer in at the layer level (two spaces shallower).
        if in_local and stripped == 'camera:':
            indent = len(line) - len(line.lstrip())
            i += 1
            while i < len(lines):
                nxt = lines[i]
                if nxt.strip() and (len(nxt) - len(nxt.lstrip())) <= indent:
                    break
                i += 1
            out.extend(VOXEL_BLOCK.format(topic=topic).rstrip('\n').split('\n'))
            removed_camera = True
            continue

        out.append(line)
        i += 1
    return '\n'.join(out), removed_camera, rewired, added


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['sim', 'real'], required=True)
    ap.add_argument('--package-root',
                    default=os.path.join(REPO, 'src', 'leo_nav2_exploration'))
    args = ap.parse_args()

    path = os.path.join(args.package_root, 'config', args.profile, 'nav2.yaml')
    if not os.path.isfile(path):
        sys.exit(f'no such file: {path}')
    text = open(path, encoding='utf-8').read()
    if 'voxel_layer' in text:
        print(f'{path}: already on VoxelLayer')
        return
    topic = ('/leo1/camera/points' if args.profile == 'sim'
             else '/camera/camera/depth/color/points')
    new_text, removed, rewired, added = convert(text, topic)
    open(path, 'w', encoding='utf-8', newline='\n').write(new_text)
    print(f'{path}: camera_source_removed={removed} sources_rewired={rewired} '
          f'voxel_added={added} topic={topic}')


if __name__ == '__main__':
    main()
