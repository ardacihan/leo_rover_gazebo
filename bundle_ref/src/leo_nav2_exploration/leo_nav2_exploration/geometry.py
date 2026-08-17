"""Robot footprint geometry shared by configuration and calibration tools."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List

import yaml


@dataclass(frozen=True)
class FootprintExtents:
    """Axis-aligned robot extents measured from the configured base frame."""

    front: float
    rear: float
    left: float
    right: float
    padding: float = 0.0

    def __post_init__(self) -> None:
        for name in ("front", "rear", "left", "right"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a finite positive distance")
        if not math.isfinite(self.padding) or self.padding < 0.0:
            raise ValueError("padding must be a finite non-negative distance")

    @property
    def physical_width(self) -> float:
        return self.left + self.right

    @property
    def physical_length(self) -> float:
        return self.front + self.rear

    @property
    def padded_width(self) -> float:
        return self.physical_width + 2.0 * self.padding

    @property
    def padded_length(self) -> float:
        return self.physical_length + 2.0 * self.padding


@dataclass(frozen=True)
class DoorwayMargin:
    """Clearance result for a footprint aligned with a doorway."""

    clear_width: float
    required_width: float
    total_clearance: float

    @property
    def per_side_clearance(self) -> float:
        return self.total_clearance / 2.0

    @property
    def passable(self) -> bool:
        return self.total_clearance > 0.0


def _clean(value: float) -> float:
    return round(float(value), 9)


def footprint_points(
    extents: FootprintExtents,
    *,
    include_padding: bool = False,
) -> List[List[float]]:
    """Return clockwise polygon points beginning at the front-left corner."""

    pad = extents.padding if include_padding else 0.0
    front = extents.front + pad
    rear = extents.rear + pad
    left = extents.left + pad
    right = extents.right + pad
    return [
        [_clean(front), _clean(left)],
        [_clean(front), _clean(-right)],
        [_clean(-rear), _clean(-right)],
        [_clean(-rear), _clean(left)],
    ]


def footprint_yaml_string(
    extents: FootprintExtents,
    *,
    include_padding: bool = False,
) -> str:
    """Return Nav2's footprint string representation."""

    return yaml.safe_dump(
        footprint_points(extents, include_padding=include_padding),
        default_flow_style=True,
        width=200,
    ).strip()


def circumscribed_radius(
    extents: FootprintExtents,
    *,
    include_padding: bool = True,
) -> float:
    """Return the smallest base-centred circle containing the footprint."""

    pad = extents.padding if include_padding else 0.0
    front = extents.front + pad
    rear = extents.rear + pad
    left = extents.left + pad
    right = extents.right + pad
    return max(
        math.hypot(front, left),
        math.hypot(front, right),
        math.hypot(rear, left),
        math.hypot(rear, right),
    )


def doorway_margin(
    clear_width: float,
    extents: FootprintExtents,
    *,
    additional_clearance_per_side: float = 0.0,
) -> DoorwayMargin:
    """Return padded and operational clearance for an aligned doorway pass.

    ``additional_clearance_per_side`` is a planning allowance beyond Nav2's
    footprint padding.  It is useful for checking whether a nominally valid
    opening also has enough tolerance for map and odometry error.
    """

    if not math.isfinite(clear_width) or clear_width <= 0.0:
        raise ValueError("clear_width must be a finite positive distance")
    if (
        not math.isfinite(additional_clearance_per_side)
        or additional_clearance_per_side < 0.0
    ):
        raise ValueError(
            "additional_clearance_per_side must be a finite non-negative distance"
        )
    required = (
        extents.padded_width + 2.0 * float(additional_clearance_per_side)
    )
    return DoorwayMargin(
        clear_width=float(clear_width),
        required_width=required,
        total_clearance=float(clear_width) - required,
    )


def compose_planar_pose(
    *,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    relative_x: float,
    relative_y: float,
    relative_yaw: float,
) -> tuple[float, float, float]:
    """Compose a local planar pose with a world/map-frame origin pose."""

    values = (origin_x, origin_y, origin_yaw, relative_x, relative_y, relative_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("planar pose values must be finite")
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    x = origin_x + cosine * relative_x - sine * relative_y
    y = origin_y + sine * relative_x + cosine * relative_y
    yaw = math.atan2(
        math.sin(origin_yaw + relative_yaw),
        math.cos(origin_yaw + relative_yaw),
    )
    return float(x), float(y), float(yaw)
