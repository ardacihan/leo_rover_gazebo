"""Pure decision helpers shared by the real-rover safety nodes."""

import math


def scan_yaw_from_transform(transform):
    """Return the yaw of a geometry_msgs TransformStamped, in radians.

    Sector logic operates on raw scan angles, which are only equal to base
    angles when the lidar is mounted unrotated.  Rover 4 yaws its lidar by pi,
    so every sector would otherwise be reflected: "front" would measure the
    physical rear.
    """
    q = transform.transform.rotation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def robust_clearance(values, outlier_points, default_clearance=0.0):
    """Return a low clearance percentile while ignoring a bounded outlier set."""
    finite_values = sorted(float(value) for value in values if value > 0.0)
    if not finite_values:
        return float(default_clearance)
    index = min(max(int(outlier_points), 0), len(finite_values) - 1)
    return finite_values[index]


def turn_clearances(left, right, rear_left, rear_right):
    """Return left/right clearances including the opposite swept rear corner."""
    return min(left, rear_right), min(right, rear_left)


def choose_turn_direction(left_clearance, right_clearance):
    """Return +1 for left or -1 for right, preferring the safer sweep."""
    return 1.0 if left_clearance >= right_clearance else -1.0


def choose_escape_action(
    left_clearance,
    right_clearance,
    minimum_turn_clearance,
    rear_clearance,
    minimum_reverse_clearance,
    reverse_attempts,
    maximum_reverse_attempts,
):
    """Choose a turn, bounded reverse, or stop for a persistent front block."""
    if max(left_clearance, right_clearance) >= minimum_turn_clearance:
        return "turn", choose_turn_direction(left_clearance, right_clearance)
    if (
        reverse_attempts < maximum_reverse_attempts
        and rear_clearance >= minimum_reverse_clearance
    ):
        return "reverse", 0.0
    return "stop", 0.0
