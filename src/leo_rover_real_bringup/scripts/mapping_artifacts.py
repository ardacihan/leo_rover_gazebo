"""Pure helpers for rendering an occupancy map with a recorded rover path."""

import binascii
import math
import struct
import zlib

import numpy as np


def world_to_grid(points_xy, origin_xy, origin_yaw, resolution):
    """Convert world/map coordinates to integer OccupancyGrid coordinates."""
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    if points.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    relative = points - np.asarray(origin_xy, dtype=np.float64).reshape(1, 2)
    cosine = math.cos(float(origin_yaw))
    sine = math.sin(float(origin_yaw))
    # OccupancyGrid coordinates use the map origin's local axes.
    local_x = cosine * relative[:, 0] + sine * relative[:, 1]
    local_y = -sine * relative[:, 0] + cosine * relative[:, 1]
    return np.floor(np.column_stack((local_x, local_y)) / resolution).astype(np.int64)


def occupancy_rgb(data, width, height):
    """Render OccupancyGrid values as an RGB image with map Y pointing up."""
    values = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
    grey = np.full(values.shape, 205, dtype=np.uint8)
    grey[(values >= 0) & (values <= 25)] = 254
    grey[values >= 65] = 0
    intermediate = (values > 25) & (values < 65)
    grey[intermediate] = np.clip(
        254.0 - values[intermediate].astype(np.float64) * 2.54, 0.0, 254.0
    ).astype(np.uint8)
    return np.repeat(np.flipud(grey)[:, :, None], 3, axis=2)


def _draw_line(image, first, second, color, radius=1):
    """Draw a clipped Bresenham line into an RGB numpy image."""
    x0, y0 = (int(value) for value in first)
    x1, y1 = (int(value) for value in second)
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    height, width = image.shape[:2]
    while True:
        left = max(x0 - radius, 0)
        right = min(x0 + radius + 1, width)
        top = max(y0 - radius, 0)
        bottom = min(y0 + radius + 1, height)
        if left < right and top < bottom:
            image[top:bottom, left:right] = color
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def render_path_overlay(
    data,
    width,
    height,
    resolution,
    origin_xy,
    origin_yaw,
    path_xy,
):
    """Return a map image with a red route, green start and blue end."""
    image = occupancy_rgb(data, width, height)
    grid_points = world_to_grid(path_xy, origin_xy, origin_yaw, resolution)
    if len(grid_points) == 0:
        return image
    # Convert bottom-left grid coordinates to top-left image coordinates.
    pixels = np.column_stack((grid_points[:, 0], int(height) - 1 - grid_points[:, 1]))
    for first, second in zip(pixels[:-1], pixels[1:]):
        _draw_line(image, first, second, (220, 30, 30), radius=1)
    _draw_line(image, pixels[0], pixels[0], (20, 170, 20), radius=2)
    _draw_line(image, pixels[-1], pixels[-1], (30, 80, 230), radius=2)
    return image


def encode_png_rgb(image):
    """Encode an uint8 RGB array as PNG without an optional image dependency."""
    array = np.asarray(image, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("PNG input must be HxWx3 RGB")
    height, width = array.shape[:2]

    def chunk(kind, payload):
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(
            ">I", binascii.crc32(body) & 0xFFFFFFFF
        )

    scanlines = b"".join(b"\x00" + row.tobytes() for row in array)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(scanlines, level=6))
        + chunk(b"IEND", b"")
    )
