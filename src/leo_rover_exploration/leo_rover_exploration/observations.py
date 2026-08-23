"""Pure helpers for deciding whether item detections are independent."""

import math

from .selection import angle_distance


def distinct_view(previous, current, min_distance, min_interval,
                  min_yaw_delta):
    """Return true when two observations are meaningfully independent.

    Views are tuples ``(time_seconds, x, y, yaw)``.  A later observation may
    qualify through elapsed time, camera translation, or a changed heading;
    adjacent frames from an unchanged pose therefore count only once.
    """
    if previous is None or current is None:
        return previous is None
    dt = max(0.0, current[0] - previous[0])
    distance = math.hypot(current[1] - previous[1],
                          current[2] - previous[2])
    yaw_delta = angle_distance(current[3], previous[3])
    return (dt >= min_interval or distance >= min_distance
            or yaw_delta >= min_yaw_delta)
