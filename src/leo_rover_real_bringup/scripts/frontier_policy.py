#!/usr/bin/env python3

"""Pure decision logic for frontier-driven exploration. No ROS imports.

The bounded explorer wanders: it drives until something is in the way, turns,
and repeats, so it re-covers ground it has already seen and stops on a timer
rather than on completion. Driving to frontiers instead means the rover heads
for places it has no data about, and "no frontiers left" is a real completion
signal.
"""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def is_blacklisted(point, blacklist, radius):
    """True if a candidate sits within `radius` of an abandoned target."""
    for bx, by in blacklist:
        if math.hypot(point[0] - bx, point[1] - by) <= radius:
            return True
    return False


def score_target(cell_count, distance, distance_weight=1.0):
    """Rank a frontier: bigger is better, farther is worse.

    Distance is penalised rather than minimised outright. Always taking the
    nearest frontier makes the rover oscillate between two small nearby gaps
    while a large unexplored region sits just past them.
    """
    return float(cell_count) / (1.0 + distance_weight * max(float(distance), 0.0))


def select_target(
    clusters,
    robot_xy,
    blacklist=(),
    blacklist_radius=0.5,
    minimum_cells=4,
    distance_weight=1.0,
):
    """Pick the best frontier to drive to, or None when exploration is done.

    `clusters` is a sequence of (cell_count, x, y) in the map frame.
    """
    best, best_score = None, 0.0
    for count, x, y in clusters:
        if count < minimum_cells:
            continue
        if is_blacklisted((x, y), blacklist, blacklist_radius):
            continue
        distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
        score = score_target(count, distance, distance_weight)
        if score > best_score:
            best, best_score = (x, y), score
    return best


def heading_error(target, robot_xy, robot_yaw):
    """Signed angle the rover must turn to face the target, in radians."""
    desired = math.atan2(target[1] - robot_xy[1], target[0] - robot_xy[0])
    return normalize_angle(desired - robot_yaw)


def distance_to(target, robot_xy):
    return math.hypot(target[0] - robot_xy[0], target[1] - robot_xy[1])


def turn_command(error, gain, max_angular, min_angular=0.10):
    """Proportional turn rate with a floor, so tiny errors still finish.

    Below the floor the wheels stall against friction instead of turning, and
    the rover sits forever a few degrees short of its heading.
    """
    magnitude = min(abs(error) * gain, max_angular)
    magnitude = max(magnitude, min_angular)
    return math.copysign(magnitude, error)


def should_abandon(
    elapsed,
    time_budget,
    progress,
    minimum_progress,
    stall_elapsed,
    stall_timeout,
):
    """Give up on a target that is taking too long or making no headway.

    Without this the rover pushes at an unreachable frontier, such as the far
    side of a glass door or a gap narrower than the chassis, until the run ends.
    """
    if elapsed >= time_budget:
        return "target time budget exhausted"
    if stall_elapsed >= stall_timeout and progress < minimum_progress:
        return f"no progress ({progress:.2f} m) for {stall_elapsed:.0f} s"
    return None
