#!/usr/bin/env python3
"""
Patch the local Husarion office world with inline AprilTag models.

Always reads the clean husarion_office.sdf source (no old tag includes).
Writes husarion_office_aruco.sdf with six inline tag planes and file:// textures.

  python3 src/multi_robot_shared_mapping/scripts/patch_husarion_office_apriltag_world.py
"""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(SCRIPT_DIR)

sys.path.insert(0, PKG_ROOT)

from multi_robot_shared_mapping.apriltag_inline_model import (  # noqa: E402
    load_tag_spawn_entries,
    make_inline_tag_world_block,
)

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_WORLD_SRC = "/ros2_ws/src/husarion_gz_worlds/worlds/husarion_office.sdf"
DEFAULT_WORLD_DST = "/ros2_ws/src/husarion_gz_worlds/worlds/husarion_office_aruco.sdf"
DEFAULT_LANDMARKS = os.path.join(PKG_ROOT, "config", "apriltag_landmarks.yaml")
DEFAULT_ASSETS = os.path.join(PKG_ROOT, "assets", "apriltags")

APRILTAG_MARKER = "<!-- AprilTags for multi_robot_shared_mapping (tag36h11) -->"


def strip_existing_apriltags(content: str) -> str:
    if APRILTAG_MARKER not in content:
        return content

    start = content.index(APRILTAG_MARKER)
    state_marker = "    <state world_name="
    state_idx = content.find(state_marker, start)
    if state_idx == -1:
        end = content.find("  </world>", start)
        if end == -1:
            raise RuntimeError("Could not find end of AprilTag block")
        return content[:start] + content[end:]
    return content[:start] + content[state_idx:]


def patch_world(
    src_path: str,
    dst_path: str,
    landmarks_path: str,
    assets_dir: str,
) -> None:
    if not os.path.isfile(src_path):
        raise FileNotFoundError(f"Source world not found: {src_path}")

    with open(src_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    content = strip_existing_apriltags(content)

    insert_marker = "    <state world_name="
    if insert_marker not in content:
        raise RuntimeError("Could not find <state world_name= marker in source SDF")

    blocks = [f"    {APRILTAG_MARKER}\n"]
    for tag_name, x, y, z, roll, pitch, yaw, texture_path, tag_size in load_tag_spawn_entries(
        landmarks_path, assets_dir
    ):
        tag_id = int(tag_name.split("_")[1])
        blocks.append(
            make_inline_tag_world_block(
                tag_name,
                tag_id,
                texture_path,
                float(x),
                float(y),
                float(z),
                float(roll),
                float(pitch),
                float(yaw),
                tag_size,
            )
        )

    patched = content.replace(insert_marker, "".join(blocks) + insert_marker, 1)

    with open(dst_path, "w", encoding="utf-8") as handle:
        handle.write(patched)

    print(f"Patched world written to: {dst_path}")


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_WORLD_SRC
    dst = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_WORLD_DST
    landmarks = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_LANDMARKS
    assets = sys.argv[4] if len(sys.argv) > 4 else DEFAULT_ASSETS
    patch_world(src, dst, landmarks, assets)
    return 0


if __name__ == "__main__":
    sys.exit(main())
