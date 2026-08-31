"""Per-world, per-robot spawn poses, and the leo2->leo1 offset they imply.

Two rovers proving collaborative mapping have to start in *different rooms*,
not 1.5 m apart at the origin -- otherwise they map the same space, the
coordination signal is nil, and the merge looks perfect for the wrong reason.

This lives in its own module because two things need it and they must not
disagree: `two_robots_gpu.launch.py` spawns from it, and the run harness scores
the recovered alignment against `relative_offset()` from it. A table copied
into the harness would drift from the table the sim actually used, and the
alignment error in the report would be quietly measuring the wrong thing.

Poses are `(x, y, z, R, P, Y)` strings. Every one was clearance-checked against
the rasterized world (`scripts/pick_spawns.py`) except husarion_office, whose
mesh collisions the rasterizer cannot see; its poses are the ones
`two_robots.launch.py` already authored and has spawned successfully.
"""

import math
import sys

SPAWN_POSES = {
    # leo2's authored pose (2.36, -11.27, z=0.05) is a trap: the 2026-08-23
    # Phase 1 run left it nose-to-a-wall, 1.3 m travelled in 25 minutes, 16.7 m2
    # mapped, and 300+ camera frames of blank plaster. Its replacement is not a
    # guess -- it is a point leo1 physically drove through in that same run
    # (scripts/pick_spawn_from_map.py scores leo1's own trajectory against the
    # map it built), 13.8 m from leo1's spawn and in the opposite corner.
    # Facing west, along the southern corridor and away from the wall.
    'husarion_office': {
        'leo1': ('0.0', '0.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('9.60', '-9.95', '0.2', '0.0', '0.0', '3.1416'),
    },
    # Corridor runs east-west through y=0; rooms are north (y>1.2) and south
    # (y<-1.2). leo1 in office N1, leo2 in office S2, diagonally opposite.
    # (-8, 5) is inside a prop, so leo1 sits at (-7, 5).
    'office_world': {
        'leo1': ('-7.0', '5.0', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('4.0', '-5.0', '0.2', '0.0', '0.0', '3.1416'),
    },
    # leo1 north-central, leo2 in the south-east room behind the x=2 partition.
    # AWS RoboMaker small house (vendored at src/aws_small_house, ros2
    # branch): ~19x11 m residence, many rooms, narrow doorways, dense
    # furniture. Spawns sit in the far west / far east rooms, facing in.
    'small_house': {
        'leo1': ('-6.5', '-2.5', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('6.5', '-2.5', '0.2', '0.0', '0.0', '3.1416'),
    },
    'small_house_l3': {
        'leo1': ('-6.5', '-2.5', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('6.5', '-2.5', '0.2', '0.0', '0.0', '3.1416'),
    },
    'small_house_l9': {
        'leo1': ('-6.5', '-2.5', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('6.5', '-2.5', '0.2', '0.0', '0.0', '3.1416'),
    },
    'small_house_l15': {
        'leo1': ('-6.5', '-2.5', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('6.5', '-2.5', '0.2', '0.0', '0.0', '3.1416'),
    },
    'depot_world': {
        'leo1': ('0.0', '4.5', '0.2', '0.0', '0.0', '0.0'),
        'leo2': ('3.0', '-4.5', '0.2', '0.0', '0.0', '3.1416'),
    },
}

# Used when a world has no authored per-room poses. Keeps the old behaviour.
FALLBACK_SPAWNS = [(0.0, 0.0), (1.5, 0.0), (0.0, 1.5), (1.5, 1.5)]


def relative_offset(world_name):
    """Ground-truth (x, y, yaw) of leo2's map origin in leo1's map frame.

    Each rover's slam_toolbox anchors its map frame at that rover's starting
    pose, so leo2/map expressed in leo1/map is exactly leo2's spawn expressed
    in leo1's spawn frame. This is what a tag-recovered transform should equal,
    and it is used for scoring only -- never fed to an aligner.
    """
    poses = SPAWN_POSES.get(world_name)
    if not poses:
        return None
    x1, y1, _, _, _, yaw1 = (float(v) for v in poses['leo1'])
    x2, y2, _, _, _, yaw2 = (float(v) for v in poses['leo2'])
    dx, dy = x2 - x1, y2 - y1
    c, s = math.cos(-yaw1), math.sin(-yaw1)
    yaw = math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1))
    return (dx * c - dy * s, dx * s + dy * c, yaw)


# Outer extent of each world, in world coordinates. Frontier detection needs
# this: a thin outer wall leaks a few lidar rays, leaving unknown cells beyond
# it that border free cells inside, and that is a perfectly valid frontier
# which no path can ever reach. On depot 2026-08-24, 43% of one rover's failed
# goals were for frontiers at x>7 in a world that ends at x=7.
WORLD_BOUNDS = {
    'small_house': (-9.5, 9.6, -5.9, 5.6),
    'small_house_l3': (-9.5, 9.6, -5.9, 5.6),
    'small_house_l9': (-9.5, 9.6, -5.9, 5.6),
    'small_house_l15': (-9.5, 9.6, -5.9, 5.6),
    'depot_world': (-7.0, 7.0, -7.0, 7.0),
    'office_world': (-12.0, 12.0, -8.0, 8.0),
    # Measured from the world's own floor meshes
    # (husarion_gz_worlds/models/Surfaces/meshes/floor_*.obj), whose union
    # spans x[0.27, 14.43] y[-13.06, 0.39] -- 162 m2 of authored floor. The
    # previous (-4, 27, -15, 4) box claimed 31 x 19 m and so let frontiers
    # up to 12 m east of the building stay eligible. One metre of margin
    # keeps real frontiers safe under SLAM drift.
    'husarion_office': (-1.0, 13.6, -12.2, 1.5),
}


def bounds_in_robot_frame(world_name, robot):
    """World extent expressed in one rover's own map frame.

    Each rover's map is anchored on its own start pose, so the shared world
    box has to be rotated into that frame. The axis-aligned box of the rotated
    corners is used, which is exact for the 0 and 180 degree spawns in use and
    conservative (slightly too generous) for anything else -- erring towards
    letting a real frontier through rather than discarding one.
    """
    box = WORLD_BOUNDS.get(world_name)
    poses = SPAWN_POSES.get(world_name)
    if not box or not poses or robot not in poses:
        return None
    sx, sy, _, _, _, syaw = (float(v) for v in poses[robot])
    c, s = math.cos(-syaw), math.sin(-syaw)
    xs, ys = [], []
    for wx in (box[0], box[1]):
        for wy in (box[2], box[3]):
            dx, dy = wx - sx, wy - sy
            xs.append(c * dx - s * dy)
            ys.append(s * dx + c * dy)
    return (min(xs), max(xs), min(ys), max(ys))


if __name__ == '__main__':
    # `spawn_poses.py <world>` prints "x y yaw" for the run harness to read.
    world = sys.argv[1] if len(sys.argv) > 1 else 'husarion_office'
    if len(sys.argv) > 2 and sys.argv[2] in ('leo1', 'leo2'):
        b = bounds_in_robot_frame(world, sys.argv[2])
        if b is None:
            sys.stderr.write('no bounds for ' + world + '/' + sys.argv[2] + chr(10))
            sys.exit(1)
        print(f'{b[0]:.3f},{b[1]:.3f},{b[2]:.3f},{b[3]:.3f}')
        sys.exit(0)
    offset = relative_offset(world)
    if offset is None:
        sys.stderr.write(f'no authored spawn poses for world {world!r}\n')
        sys.exit(1)
    print(f'{offset[0]:.4f} {offset[1]:.4f} {offset[2]:.4f}')
