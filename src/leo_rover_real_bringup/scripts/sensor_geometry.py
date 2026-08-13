"""Pure geometry helpers for depth projection and planar scan fusion."""

import math

import numpy as np


def quaternion_rotation_matrix(quaternion_xyzw):
    """Return a normalized 3x3 rotation matrix for an ``(x, y, z, w)`` quaternion."""
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def project_depth_to_base(
    depth_m,
    fx,
    fy,
    cx,
    cy,
    rotation,
    translation,
    pixel_stride=2,
    valid_depth_min=0.10,
    valid_depth_max=10.0,
):
    """Project a metric optical-frame depth image into a base-frame point array.

    The optical convention is X right, Y down, Z forward. ``rotation`` and
    ``translation`` must describe ``base_frame <- optical_frame``.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth image must be a 2D array")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    stride = max(int(pixel_stride), 1)
    rows = np.arange(0, depth.shape[0], stride, dtype=np.int32)
    columns = np.arange(0, depth.shape[1], stride, dtype=np.int32)
    sampled = depth[np.ix_(rows, columns)]
    valid = (
        np.isfinite(sampled)
        & (sampled >= float(valid_depth_min))
        & (sampled <= float(valid_depth_max))
    )
    valid_fraction = float(np.count_nonzero(valid)) / float(valid.size or 1)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64), valid_fraction

    grid_u, grid_v = np.meshgrid(columns, rows)
    z_forward = sampled[valid].astype(np.float64)
    x_right = (grid_u[valid].astype(np.float64) - float(cx)) * z_forward / float(fx)
    y_down = (grid_v[valid].astype(np.float64) - float(cy)) * z_forward / float(fy)
    optical_points = np.column_stack((x_right, y_down, z_forward))

    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    offset = np.asarray(translation, dtype=np.float64).reshape(1, 3)
    return optical_points @ matrix.T + offset, valid_fraction


def points_to_scan_ranges(
    points,
    min_height,
    max_height,
    angle_min,
    angle_max,
    angle_increment,
    range_min,
    range_max,
):
    """Height-filter base-frame points and return nearest planar range per bin."""
    if angle_increment <= 0.0 or angle_max <= angle_min:
        raise ValueError("invalid scan angular limits")
    count = int(math.floor((angle_max - angle_min) / angle_increment)) + 1
    ranges = np.full(count, np.inf, dtype=np.float32)
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.size == 0:
        return ranges
    cloud = cloud.reshape(-1, 3)
    planar_range = np.hypot(cloud[:, 0], cloud[:, 1])
    angle = np.arctan2(cloud[:, 1], cloud[:, 0])
    keep = (
        np.all(np.isfinite(cloud), axis=1)
        & (cloud[:, 2] >= float(min_height))
        & (cloud[:, 2] <= float(max_height))
        & (planar_range >= float(range_min))
        & (planar_range <= float(range_max))
        & (angle >= float(angle_min))
        & (angle <= float(angle_max))
    )
    if not np.any(keep):
        return ranges
    indices = np.floor((angle[keep] - angle_min) / angle_increment).astype(np.int64)
    indices = np.clip(indices, 0, count - 1)
    np.minimum.at(ranges, indices, planar_range[keep].astype(np.float32))
    return ranges


def scan_to_base_points(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    rotation,
    translation,
    self_mask_angle_min=None,
    self_mask_angle_max=None,
    self_mask_max_range=0.0,
    self_mask_footprint_radius=0.0,
):
    """Convert scan samples into base-frame XYZ points with an optional bounded self-mask.

    The angular mask covers the known mast signature. `self_mask_footprint_radius`
    is a backstop for structure outside that window: any return landing inside
    the rover's own footprint cannot be an external obstacle, whatever its
    bearing. Rover 4 needs it for a bracket return at raw-laser -142..-140 deg
    that the mast window does not reach.
    """
    values = np.asarray(ranges, dtype=np.float64)
    angles = float(angle_min) + np.arange(values.size, dtype=np.float64) * float(angle_increment)
    valid = (
        np.isfinite(values)
        & (values >= float(range_min))
        & (values <= float(range_max))
    )
    if self_mask_angle_min is not None and self_mask_angle_max is not None:
        self_return = (
            (angles >= float(self_mask_angle_min))
            & (angles <= float(self_mask_angle_max))
            & np.isfinite(values)
            & (values <= float(self_mask_max_range))
        )
        valid &= ~self_return
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)
    sensor_points = np.column_stack((
        values[valid] * np.cos(angles[valid]),
        values[valid] * np.sin(angles[valid]),
        np.zeros(np.count_nonzero(valid), dtype=np.float64),
    ))
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    offset = np.asarray(translation, dtype=np.float64).reshape(1, 3)
    base_points = sensor_points @ matrix.T + offset
    radius = float(self_mask_footprint_radius)
    if radius > 0.0:
        outside = np.hypot(base_points[:, 0], base_points[:, 1]) > radius
        base_points = base_points[outside]
    return base_points


def merge_planar_points(
    point_sets,
    angle_min,
    angle_max,
    angle_increment,
    range_min,
    range_max,
):
    """Merge already-base-frame point sets into a planar nearest-return scan."""
    nonempty = [np.asarray(points).reshape(-1, 3) for points in point_sets if np.asarray(points).size]
    points = np.vstack(nonempty) if nonempty else np.empty((0, 3), dtype=np.float64)
    return points_to_scan_ranges(
        points,
        min_height=-math.inf,
        max_height=math.inf,
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
    )
