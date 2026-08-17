import math

import pytest
import yaml

from leo_nav2_exploration.geometry import (
    FootprintExtents,
    circumscribed_radius,
    doorway_margin,
    footprint_points,
    footprint_yaml_string,
)


def test_centered_footprint_points_use_nav2_clockwise_order():
    extents = FootprintExtents(
        front=0.21,
        rear=0.21,
        left=0.21,
        right=0.21,
        padding=0.01,
    )

    assert footprint_points(extents, include_padding=False) == [
        [0.21, 0.21],
        [0.21, -0.21],
        [-0.21, -0.21],
        [-0.21, 0.21],
    ]
    assert footprint_points(extents, include_padding=True) == [
        [0.22, 0.22],
        [0.22, -0.22],
        [-0.22, -0.22],
        [-0.22, 0.22],
    ]


def test_asymmetric_footprint_preserves_independent_extents():
    extents = FootprintExtents(
        front=0.24,
        rear=0.18,
        left=0.23,
        right=0.20,
        padding=0.0,
    )

    assert footprint_points(extents) == [
        [0.24, 0.23],
        [0.24, -0.20],
        [-0.18, -0.20],
        [-0.18, 0.23],
    ]


def test_footprint_yaml_string_round_trips_as_list():
    extents = FootprintExtents(0.21, 0.21, 0.21, 0.21, 0.01)

    parsed = yaml.safe_load(footprint_yaml_string(extents))

    assert parsed == footprint_points(extents, include_padding=False)


def test_circumscribed_radius_uses_farthest_corner_and_padding():
    extents = FootprintExtents(0.24, 0.18, 0.23, 0.20, 0.01)

    assert circumscribed_radius(extents, include_padding=False) == pytest.approx(
        math.hypot(0.24, 0.23)
    )
    assert circumscribed_radius(extents, include_padding=True) == pytest.approx(
        math.hypot(0.25, 0.24)
    )


def test_doorway_margin_reports_total_and_each_side_clearance():
    extents = FootprintExtents(0.21, 0.21, 0.21, 0.21, 0.01)

    margin = doorway_margin(clear_width=0.78, extents=extents)

    assert margin.required_width == pytest.approx(0.44)
    assert margin.total_clearance == pytest.approx(0.34)
    assert margin.per_side_clearance == pytest.approx(0.17)
    assert margin.passable is True


def test_doorway_margin_can_add_operational_clearance():
    extents = FootprintExtents(0.21, 0.21, 0.21, 0.21, 0.01)

    margin = doorway_margin(
        clear_width=0.50,
        extents=extents,
        additional_clearance_per_side=0.04,
    )

    assert margin.required_width == pytest.approx(0.52)
    assert margin.total_clearance == pytest.approx(-0.02)
    assert margin.passable is False


@pytest.mark.parametrize(
    "kwargs",
    [
        {"front": 0.0, "rear": 0.2, "left": 0.2, "right": 0.2},
        {"front": 0.2, "rear": -0.1, "left": 0.2, "right": 0.2},
        {"front": 0.2, "rear": 0.2, "left": 0.2, "right": 0.2, "padding": -0.01},
    ],
)
def test_invalid_footprint_dimensions_are_rejected(kwargs):
    with pytest.raises(ValueError):
        FootprintExtents(**kwargs)


def test_invalid_doorway_width_is_rejected():
    extents = FootprintExtents(0.21, 0.21, 0.21, 0.21)

    with pytest.raises(ValueError):
        doorway_margin(0.0, extents)
