#!/usr/bin/env python3

"""Pure trigger logic deciding when a debug capture is worth saving. No ROS.

Saving every frame buries the interesting moment in thousands of identical
ones. These triggers each answer a different debugging question:

- monitor_veto   : geometry made Collision Monitor override the command
- gate_block     : a health check refused motion, which is not geometry at all
- near_miss      : the rover kept going but got close, so thresholds need work
- sensor_conflict: camera and lidar disagree, which points at fusion or calibration
- heartbeat      : baseline context so quiet stretches are not blind
"""

VELOCITY_EPSILON = 0.005


def monitor_veto(requested_linear, output_linear, reduction_threshold=0.02):
    """Collision Monitor materially reduced or cancelled a real command."""
    if abs(requested_linear) <= VELOCITY_EPSILON:
        return False
    return (abs(requested_linear) - abs(output_linear)) >= reduction_threshold


def gate_block(requested_linear, requested_angular, raw_linear, raw_angular):
    """A command was asked for but the gate emitted nothing."""
    asked = (abs(requested_linear) > VELOCITY_EPSILON
             or abs(requested_angular) > VELOCITY_EPSILON)
    emitted = (abs(raw_linear) > VELOCITY_EPSILON
               or abs(raw_angular) > VELOCITY_EPSILON)
    return asked and not emitted


def near_miss(front_clearance, threshold=0.50):
    return 0.0 < float(front_clearance) < float(threshold)


def sensor_conflict(lidar_front, camera_front, threshold=0.40, valid_max=3.0):
    """Camera and lidar disagree about the front, within camera range.

    Beyond the depth camera's usable range the two legitimately diverge, so
    comparing there would fire constantly and mean nothing.
    """
    if lidar_front <= 0.0 or camera_front <= 0.0:
        return False
    if min(lidar_front, camera_front) > valid_max:
        return False
    return abs(lidar_front - camera_front) >= threshold


def should_capture(now, last_capture_times, kind, minimum_interval):
    """Rate-limit each trigger kind independently.

    Per-kind limiting matters: a sustained veto must not crowd out a
    simultaneous sensor conflict, which is a different failure.
    """
    last = last_capture_times.get(kind)
    return last is None or (now - last) >= minimum_interval
