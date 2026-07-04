#!/usr/bin/env python3
"""Shared inline AprilTag SDF helpers (no model:// URIs)."""

from __future__ import annotations

import os
from typing import List, Tuple

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_TAG_SIZE_M = 0.35


def texture_file_uri(texture_path: str) -> str:
    return f"file://{os.path.abspath(texture_path)}"


def make_inline_tag_sdf(
    tag_name: str,
    tag_id: int,
    texture_path: str,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
) -> str:
    texture_uri = texture_file_uri(texture_path)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{tag_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 0 0 1.5708 0</pose>
        <geometry>
          <box>
            <size>{tag_size_m} 0.001 {tag_size_m}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>{texture_uri}</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0 0 1.5708 0</pose>
        <geometry>
          <box>
            <size>{tag_size_m} 0.001 {tag_size_m}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""


def make_inline_tag_world_block(
    tag_name: str,
    tag_id: int,
    texture_path: str,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
) -> str:
    texture_uri = texture_file_uri(texture_path)
    return f"""    <model name="{tag_name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <visual name="visual">
          <pose>0 0 0 0 1.5708 0</pose>
          <geometry>
            <box>
              <size>{tag_size_m} 0.001 {tag_size_m}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr>
              <metal>
                <albedo_map>{texture_uri}</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
"""


def load_tag_spawn_entries(
    landmarks_path: str,
    assets_dir: str,
) -> List[Tuple[str, str, str, str, str, str, str, str, float]]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read apriltag_landmarks.yaml")

    with open(landmarks_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    entries = []
    for tag_name in sorted(data.get("tags", {}).keys()):
        tag = data["tags"][tag_name]
        tag_id = int(tag["id"])
        texture_path = os.path.join(assets_dir, f"tag36h11_{tag_id}.png")
        entries.append((
            tag_name,
            str(tag["x"]),
            str(tag["y"]),
            str(tag["z"]),
            str(tag.get("roll", 0.0)),
            str(tag.get("pitch", 0.0)),
            str(tag.get("yaw", 0.0)),
            texture_path,
            float(tag.get("size", DEFAULT_TAG_SIZE_M)),
        ))
    return entries
