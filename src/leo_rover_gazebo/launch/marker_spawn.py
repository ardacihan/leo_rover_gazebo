"""Spawn wall-mounted ArUco markers into a running Gazebo world.

`office_world.sdf` already carries correct markers, baked in: a white backing
board for the quiet zone, then a 0.20 m textured black square on a plate that
is thin along **x**, so its face normal is `(cos yaw, sin yaw)` -- the
convention the `mock_markers_*.yaml` files document as "yaw = outward wall
normal". This module reproduces that geometry as an inline SDF for the worlds
that do not have it (husarion_office has no markers at all), so there is one
marker convention in the tree rather than two.

Two things it does differently from the baked-in markers, both deliberate:

* **`marker_length` is the plate side, 0.20 m, with nothing to derive.** The
  standalone `models/aruco_N/` textures carry no quiet zone -- measured, the
  black square fills 100% of every one -- so padding it into the geometry is
  the only way the number stays honest.
* **The plate is textured on both faces.** A marker taped to a real wall is
  one-sided, but a yaw that is 180 degrees out points the readable face into
  the wall and the marker silently never detects. Facing is the one thing
  about the ported husarion poses that the source table does not settle, so
  both faces carry the texture and the wall itself does the occluding.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node

# Side of the black square, metres. Hand this to `aruco_detector` verbatim as
# `marker_length`; getting it wrong scales every tag pose along the view ray
# without erroring.
MARKER_LENGTH = 0.20
# One dictionary cell of white quiet zone each side: DICT_4X4_50 markers are
# 4 payload + 1 black border cells per side, so the board is 8/6 of the plate.
BOARD_SIDE = MARKER_LENGTH * 8.0 / 6.0


def marker_ground_truth(world_name):
    """[(id, x, y, z, yaw)] for a world, from the shared mock-marker yaml.

    Returns [] when a world has no authored markers -- callers spawn nothing
    rather than guessing positions.
    """
    try:
        config_dir = os.path.join(
            get_package_share_directory('leo_rover_exploration'), 'config')
    except Exception:
        return []
    path = os.path.join(config_dir, f'mock_markers_{world_name}.yaml')
    if not os.path.exists(path):
        return []
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    out = []
    for m in data.get('markers', []):
        out.append((int(m['id']), float(m['x']), float(m['y']),
                    float(m.get('z', 0.3)), float(m.get('yaw', 0.0))))
    return out


def _texture_uri(marker_id):
    """`aruco_N_bare.png` is the bare black square, which is what the plate
    geometry assumes. Fall back to the padded texture only if it is missing."""
    models = os.path.join(get_package_share_directory('leo_rover_gazebo'),
                          'models', 'aruco_markers', 'textures')
    bare = f'aruco_{marker_id}_bare.png'
    if os.path.exists(os.path.join(models, bare)):
        return f'model://aruco_markers/textures/{bare}'
    return f'model://aruco_markers/textures/aruco_{marker_id}.png'


def marker_sdf(marker_id):
    """Inline SDF for one marker: white board, plate textured on both faces."""
    uri = _texture_uri(marker_id)

    def face(name, x):
        return f'''
        <visual name="{name}">
          <pose>{x} 0 0 0 0 0</pose>
          <geometry><box><size>0.002 {MARKER_LENGTH} {MARKER_LENGTH}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal>
              <albedo_map>{uri}</albedo_map>
              <metalness>0.0</metalness><roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>'''

    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="aruco_marker_{marker_id}">
    <static>true</static>
    <link name="link">
      <visual name="board">
        <geometry><box><size>0.01 {BOARD_SIDE:.4f} {BOARD_SIDE:.4f}</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>{face('front', 0.006)}{face('back', -0.006)}
    </link>
  </model>
</sdf>'''


def marker_spawn_actions(world_name, period=10.0, stagger=0.4):
    """TimerActions that spawn every authored marker for `world_name`.

    Staggered because `ros_gz_sim create` calls are synchronous service calls
    into the same server that is still loading the world and both rovers.
    """
    actions = []
    for index, (mid, x, y, z, yaw) in enumerate(marker_ground_truth(world_name)):
        actions.append(TimerAction(
            period=period + index * stagger,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_aruco_{mid}',
                arguments=[
                    '-name', f'aruco_marker_{mid}',
                    '-x', str(x), '-y', str(y), '-z', str(z),
                    '-R', '0', '-P', '0', '-Y', str(yaw),
                    '-string', marker_sdf(mid),
                ],
                output='screen',
            )],
        ))
    return actions
