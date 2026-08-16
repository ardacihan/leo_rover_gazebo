"""Pure scoring helpers used by camera sweep planning."""

import math


def angle_distance(a, b):
    """Return the smallest absolute angular distance between two headings."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def sweep_candidate_metrics(robot, robot_yaw, viewpoint, target, gain,
                            heading_weight=0.75):
    """Score a viewpoint by expected new information per motion cost.

    The old planner selected the largest wall cluster even when it was on the
    opposite side of the map.  This utility keeps useful nearby work local,
    while still allowing a sufficiently large remote cluster to win.
    """
    dx = viewpoint[0] - robot[0]
    dy = viewpoint[1] - robot[1]
    distance = math.hypot(dx, dy)
    travel_yaw = math.atan2(dy, dx) if distance > 1e-6 else robot_yaw
    camera_yaw = math.atan2(target[1] - viewpoint[1],
                            target[0] - viewpoint[0])
    heading_cost = (angle_distance(robot_yaw, travel_yaw)
                    + 0.5 * angle_distance(travel_yaw, camera_yaw))
    motion_cost = 1.0 + distance + heading_weight * heading_cost
    return {
        'gain': int(gain),
        'distance': distance,
        'heading_cost': heading_cost,
        'utility': float(gain) / motion_cost,
    }
