#!/usr/bin/env python3
"""Generate a large, clean multi-room world for the collaborative-exploration
2x demonstration. A 30x24 m building split into a 3x3 grid of 10x8 m rooms by
internal walls with wide (2 m) doorways. No furniture (fast, wedge-free with
lidar only); the origin sits in the open centre room (spawn-safe). Distinct
room positions give SLAM enough structure.

Writes src/leo_rover_gazebo/worlds/big_world.sdf.
"""
import os

HALF_W, HALF_H = 15.0, 12.0      # world spans x[-15,15], y[-12,12]
COL_X = [-5.0, 5.0]              # vertical divider walls
ROW_Y = [-4.0, 4.0]             # horizontal divider walls
ROOM_CX = [-10.0, 0.0, 10.0]    # room centres (for doorway gaps)
ROOM_CY = [-8.0, 0.0, 8.0]
DOOR = 2.0                       # doorway width
TH = 0.2                         # wall thickness
H = 2.0                          # wall height

walls = []  # (name, cx, cy, sx, sy)


def add(name, cx, cy, sx, sy):
    walls.append((name, cx, cy, sx, sy))


# Outer shell
add('outer_n', 0, HALF_H, 2 * HALF_W + TH, TH)
add('outer_s', 0, -HALF_H, 2 * HALF_W + TH, TH)
add('outer_e', HALF_W, 0, TH, 2 * HALF_H + TH)
add('outer_w', -HALF_W, 0, TH, 2 * HALF_H + TH)


def segments(lo, hi, gaps, half_door):
    """Solid intervals of [lo,hi] with a gap of +-half_door around each gap."""
    cuts = sorted(gaps)
    segs = []
    x = lo
    for g in cuts:
        a, b = g - half_door, g + half_door
        if a > x:
            segs.append((x, a))
        x = max(x, b)
    if x < hi:
        segs.append((x, hi))
    return segs


# Vertical divider walls (constant x), gaps at each room-centre y
for i, x in enumerate(COL_X):
    for j, (a, b) in enumerate(segments(-HALF_H, HALF_H, ROOM_CY, DOOR / 2)):
        add(f'vwall_{i}_{j}', x, (a + b) / 2, TH, b - a)

# Horizontal divider walls (constant y), gaps at each room-centre x
for i, y in enumerate(ROW_Y):
    for j, (a, b) in enumerate(segments(-HALF_W, HALF_W, ROOM_CX, DOOR / 2)):
        add(f'hwall_{i}_{j}', (a + b) / 2, y, b - a, TH)


def wall_sdf(name, cx, cy, sx, sy):
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {H/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {H}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>
'''


HEADER = '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="big_world">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>2.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
'''
FOOTER = '  </world>\n</sdf>\n'

out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'src', 'leo_rover_gazebo', 'worlds', 'big_world.sdf')
with open(out, 'w') as f:
    f.write(HEADER)
    for w in walls:
        f.write(wall_sdf(*w))
    f.write(FOOTER)
print(f'wrote {out} with {len(walls)} wall segments '
      f'(30x24 m, 3x3 rooms, {DOOR} m doorways)')
