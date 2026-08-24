#!/usr/bin/env python3
"""Make depot_world's ArUco markers real, and add two in the shared area.

depot_world shipped four `marker_N` models that are flat magenta 0.15 m tiles
with no texture -- placeholders for the *mock* detector, invisible to the real
one. office_world already carries the correct pattern (white backing board for
the quiet zone + a 0.20 m textured black square on a plate thin along x, normal
= (cos yaw, sin yaw)); this rewrites depot's four to match it.

It also adds ids 5 and 6. The original four are one-per-room: leo1 (spawning
north-central) and leo2 (spawning in the south-east room) can each see two, but
never the *same* two, and tag alignment needs common landmarks. Ids 5 and 6 sit
in the south-central area both rovers must cross, giving three mutually visible
tags (3, 5, 6) with several metres of spread.
"""

import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'worlds', 'depot_world.sdf')
YAML = os.path.join(ROOT, 'src', 'leo_rover_exploration', 'config',
                    'mock_markers_depot_world.yaml')

# id -> (x, y, z, yaw, comment)
MARKERS = {
    1: (0.0, 6.88, 0.3, -1.5708, 'north outer wall, north zone'),
    2: (3.0, -6.88, 0.3, 1.5708, 'south outer wall, south-east room'),
    3: (1.88, 0.0, 0.3, 3.1416, 'x=2 partition west face, central column'),
    4: (6.88, 0.0, 0.3, 3.1416, 'east outer wall, middle-east room'),
    5: (1.88, -2.0, 0.3, 3.1416, 'x=2 partition west face, south-central (shared)'),
    6: (0.0, -6.88, 0.3, 1.5708, 'south outer wall, south-central (shared)'),
}


def model_block(mid, x, y, z, yaw, comment):
    return f'''    <model name="marker_{mid}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <!-- {comment} -->
        <!-- White backing board: an ArUco marker needs a light quiet zone
             around the black square or detectMarkers never finds the quad.
             Making it geometry rather than part of the texture keeps
             marker_length equal to the plate side, with nothing to derive. -->
        <visual name="board">
          <pose>-0.005 0 0 0 0 0</pose>
          <geometry><box><size>0.01 0.30 0.30</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <!-- The marker itself: 0.20 m black square, edge to edge. -->
        <visual name="visual">
          <geometry><box><size>0.01 0.20 0.20</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal>
              <albedo_map>model://aruco_markers/textures/aruco_{mid}_bare.png</albedo_map>
              <metalness>0.0</metalness><roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>
'''


def main():
    text = open(WORLD).read()

    # Drop every existing marker_N model, then re-emit all six together.
    text, n = re.subn(r'[ \t]*<model name="marker_\d+">.*?</model>\n',
                      '', text, flags=re.S)
    print(f'removed {n} placeholder marker models')

    blocks = ''.join(model_block(mid, *MARKERS[mid]) for mid in sorted(MARKERS))
    # Insert before the closing </world>.
    idx = text.rindex('</world>')
    text = text[:idx] + blocks + text[idx:]
    open(WORLD, 'w').write(text)
    print(f'wrote {len(MARKERS)} textured markers into {WORLD}')

    lines = [
        '# Ground-truth ArUco marker poses for depot_world.',
        '# yaw = outward normal of the wall face the marker is mounted on,',
        '# i.e. the face normal is (cos yaw, sin yaw). Kept in sync with the',
        '# marker_N models in worlds/depot_world.sdf by',
        '# scripts/upgrade_depot_markers.py.',
        'markers:',
    ]
    for mid in sorted(MARKERS):
        x, y, z, yaw, comment = MARKERS[mid]
        lines.append(f'  - {{id: {mid}, x: {x}, y: {y}, z: {z}, yaw: {yaw}}}'
                     f'     # {comment}')
    open(YAML, 'w').write('\n'.join(lines) + '\n')
    print(f'wrote {YAML}')


if __name__ == '__main__':
    main()
