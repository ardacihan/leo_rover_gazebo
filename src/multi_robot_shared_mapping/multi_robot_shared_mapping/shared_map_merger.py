#!/usr/bin/env python3
"""
Simple real-time occupancy-grid merger for two robots.

First version assumption:
- Each robot runs its own SLAM node.
- Robot 1 produces /leo1/map.
- Robot 2 produces /leo2/map.
- The transform from robot2's map frame into robot1/shared map frame is known.

Later extension:
- Replace the static robot2_to_shared transform with an ArUco-based alignment provider.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


def yaw_to_quaternion(yaw: float):
    """Return z,w quaternion values for a planar yaw."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SharedMapMerger(Node):
    def __init__(self):
        super().__init__("shared_map_merger")

        self.declare_parameter("map1_topic", "/leo1/map")
        self.declare_parameter("map2_topic", "/leo2/map")
        self.declare_parameter("shared_map_topic", "/shared_map")
        self.declare_parameter("shared_frame_id", "shared_map")

        # Transform from robot2 map coordinates into shared/robot1 map coordinates.
        # For first simulation version, set this from known spawn offset.
        # Later this can come from ArUco alignment.
        self.declare_parameter("robot2_to_shared_x", 0.0)
        self.declare_parameter("robot2_to_shared_y", 0.0)
        self.declare_parameter("robot2_to_shared_yaw", 0.0)

        # Unknown = -1, free = 0, occupied = 100.
        self.declare_parameter("occupied_threshold", 50)

        self.map1: Optional[OccupancyGrid] = None
        self.map2: Optional[OccupancyGrid] = None

        map1_topic = self.get_parameter("map1_topic").value
        map2_topic = self.get_parameter("map2_topic").value
        shared_topic = self.get_parameter("shared_map_topic").value

        self.create_subscription(OccupancyGrid, map1_topic, self.map1_callback, 10)
        self.create_subscription(OccupancyGrid, map2_topic, self.map2_callback, 10)
        self.shared_pub = self.create_publisher(OccupancyGrid, shared_topic, 10)

        self.timer = self.create_timer(1.0, self.publish_shared_map)

        self.get_logger().info(f"Listening to {map1_topic} and {map2_topic}")
        self.get_logger().info(f"Publishing shared map on {shared_topic}")

    def map1_callback(self, msg: OccupancyGrid):
        self.map1 = msg

    def map2_callback(self, msg: OccupancyGrid):
        self.map2 = msg

    def grid_to_world(self, grid: OccupancyGrid, ix: int, iy: int) -> Tuple[float, float]:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return ox + (ix + 0.5) * res, oy + (iy + 0.5) * res

    def world_to_grid(self, grid: OccupancyGrid, x: float, y: float) -> Tuple[int, int]:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        ix = int(math.floor((x - ox) / res))
        iy = int(math.floor((y - oy) / res))
        return ix, iy

    def transform_robot2_to_shared(self, x: float, y: float) -> Tuple[float, float]:
        tx = float(self.get_parameter("robot2_to_shared_x").value)
        ty = float(self.get_parameter("robot2_to_shared_y").value)
        yaw = float(self.get_parameter("robot2_to_shared_yaw").value)

        c = math.cos(yaw)
        s = math.sin(yaw)

        xs = c * x - s * y + tx
        ys = s * x + c * y + ty
        return xs, ys

    def map_bounds_in_shared(self, grid: OccupancyGrid, is_robot2: bool):
        corners = [
            (0, 0),
            (grid.info.width, 0),
            (0, grid.info.height),
            (grid.info.width, grid.info.height),
        ]

        points = []
        for ix, iy in corners:
            x = grid.info.origin.position.x + ix * grid.info.resolution
            y = grid.info.origin.position.y + iy * grid.info.resolution
            if is_robot2:
                x, y = self.transform_robot2_to_shared(x, y)
            points.append((x, y))

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return min(xs), min(ys), max(xs), max(ys)

    def merge_cell(self, current: int, incoming: int) -> int:
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)

        # Keep known data over unknown data.
        if incoming < 0:
            return current
        if current < 0:
            return incoming

        # Occupied wins over free, because obstacle safety is more important.
        if incoming >= occupied_threshold or current >= occupied_threshold:
            return 100

        # Otherwise keep free.
        return 0

    def publish_shared_map(self):
        if self.map1 is None and self.map2 is None:
            return

        maps = []
        if self.map1 is not None:
            maps.append((self.map1, False))
        if self.map2 is not None:
            maps.append((self.map2, True))

        # Use the finest available resolution.
        resolution = min(m.info.resolution for m, _ in maps)

        bounds = [self.map_bounds_in_shared(m, is_r2) for m, is_r2 in maps]
        min_x = min(b[0] for b in bounds)
        min_y = min(b[1] for b in bounds)
        max_x = max(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)

        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))

        shared = OccupancyGrid()
        shared.header = Header()
        shared.header.stamp = self.get_clock().now().to_msg()
        shared.header.frame_id = str(self.get_parameter("shared_frame_id").value)
        shared.info.resolution = resolution
        shared.info.width = width
        shared.info.height = height
        shared.info.origin.position.x = min_x
        shared.info.origin.position.y = min_y
        shared.info.origin.position.z = 0.0
        shared.info.origin.orientation.w = 1.0
        shared.data = [-1] * (width * height)

        for grid, is_robot2 in maps:
            for iy in range(grid.info.height):
                for ix in range(grid.info.width):
                    value = grid.data[iy * grid.info.width + ix]
                    if value < 0:
                        continue

                    x, y = self.grid_to_world(grid, ix, iy)
                    if is_robot2:
                        x, y = self.transform_robot2_to_shared(x, y)

                    sx = int(math.floor((x - min_x) / resolution))
                    sy = int(math.floor((y - min_y) / resolution))

                    if 0 <= sx < width and 0 <= sy < height:
                        idx = sy * width + sx
                        shared.data[idx] = self.merge_cell(shared.data[idx], value)

        self.shared_pub.publish(shared)


def main(args=None):
    rclpy.init(args=args)
    node = SharedMapMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
