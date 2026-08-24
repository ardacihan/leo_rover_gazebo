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


if __name__ == '__main__':
    # `spawn_poses.py <world>` prints "x y yaw" for the run harness to read.
    world = sys.argv[1] if len(sys.argv) > 1 else 'husarion_office'
    offset = relative_offset(world)
    if offset is None:
        sys.stderr.write(f'no authored spawn poses for world {world!r}\n')
        sys.exit(1)
    print(f'{offset[0]:.4f} {offset[1]:.4f} {offset[2]:.4f}')
