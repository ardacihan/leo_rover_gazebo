"""Command-line footprint and doorway-clearance calculator."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from .geometry import (
    FootprintExtents,
    circumscribed_radius,
    doorway_margin,
    footprint_points,
    footprint_yaml_string,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a Nav2 polygon footprint from base-frame extents."
    )
    parser.add_argument("--front", type=float, default=0.21, help="Base frame to front edge, metres")
    parser.add_argument("--rear", type=float, default=0.21, help="Base frame to rear edge, metres")
    parser.add_argument("--left", type=float, default=0.21, help="Base frame to left edge, metres")
    parser.add_argument("--right", type=float, default=0.21, help="Base frame to right edge, metres")
    parser.add_argument("--padding", type=float, default=0.01, help="Nav2 footprint padding, metres")
    parser.add_argument("--door-width", type=float, help="Measured clear doorway width, metres")
    parser.add_argument("--write-yaml", type=Path, help="Optional file for a reusable geometry snippet")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        extents = FootprintExtents(
            front=args.front,
            rear=args.rear,
            left=args.left,
            right=args.right,
            padding=args.padding,
        )
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    result = {
        "front_extent": extents.front,
        "rear_extent": extents.rear,
        "left_extent": extents.left,
        "right_extent": extents.right,
        "footprint": footprint_yaml_string(extents, include_padding=False),
        "footprint_padding": extents.padding,
        "physical_width": extents.physical_width,
        "physical_length": extents.physical_length,
        "padded_width": extents.padded_width,
        "padded_length": extents.padded_length,
        "circumscribed_radius_with_padding": circumscribed_radius(extents),
    }
    if args.door_width is not None:
        margin = doorway_margin(args.door_width, extents)
        result.update(
            {
                "door_clear_width": args.door_width,
                "door_required_width": margin.required_width,
                "door_total_margin": margin.total_clearance,
                "door_margin_per_side_when_centered": margin.per_side_clearance,
                "door_fits": margin.passable,
            }
        )

    print(yaml.safe_dump(result, sort_keys=False, width=140).rstrip())
    print("\nNav2 costmap values:")
    print(f'footprint: "{result["footprint"]}"')
    print(f"footprint_padding: {extents.padding:.4f}")

    if args.write_yaml:
        snippet = {
            "geometry": {
                "front": extents.front,
                "rear": extents.rear,
                "left": extents.left,
                "right": extents.right,
                "padding": extents.padding,
                "footprint_points": footprint_points(extents),
            }
        }
        args.write_yaml.parent.mkdir(parents=True, exist_ok=True)
        args.write_yaml.write_text(yaml.safe_dump(snippet, sort_keys=False), encoding="utf-8")
        print(f"Wrote {args.write_yaml}")

    if args.door_width is not None and result["door_total_margin"] <= 0.0:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
