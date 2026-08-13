#!/usr/bin/env python3

"""Report map coverage and remaining frontiers.

Read-only by construction: it publishes no velocity and drives nothing. Use it
to decide whether a room is actually finished before ending a mapping run.
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from map_coverage import (
    cluster_frontiers,
    cluster_to_map_xy,
    coverage_stats,
    frontier_mask,
)


class MapCoverageReporter(Node):
    """Log a coverage and frontier summary on a timer."""

    def __init__(self):
        super().__init__("map_coverage_reporter")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("minimum_cluster_cells", 4)
        self.declare_parameter("report_period", 5.0)
        self.declare_parameter("reported_clusters", 5)

        self.minimum_cluster_cells = int(
            self.get_parameter("minimum_cluster_cells").value
        )
        self.reported_clusters = int(
            self.get_parameter("reported_clusters").value
        )
        self.latest = None

        # SLAM Toolbox latches /map, so a volatile subscription can sit silent
        # until the next update. Match its transient-local durability.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value,
            self._map_callback, qos
        )
        self.create_timer(
            float(self.get_parameter("report_period").value), self._report
        )

    def _map_callback(self, msg):
        self.latest = msg

    def _report(self):
        if self.latest is None:
            self.get_logger().warn("no map received yet")
            return
        info = self.latest.info
        cells = np.asarray(self.latest.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        unknown, free, occupied = coverage_stats(cells)
        area = free * info.width * info.height * info.resolution ** 2
        clusters = cluster_frontiers(
            frontier_mask(cells), self.minimum_cluster_cells
        )
        self.get_logger().info(
            f"map {info.width}x{info.height} @ {info.resolution:.3f} m | "
            f"unknown {100*unknown:.1f}% free {100*free:.1f}% "
            f"occupied {100*occupied:.1f}% | mapped {area:.1f} m^2 | "
            f"{len(clusters)} frontier clusters"
        )
        for count, row, col in clusters[: self.reported_clusters]:
            x, y = cluster_to_map_xy(
                row, col, info.origin.position.x, info.origin.position.y,
                info.resolution
            )
            self.get_logger().info(
                f"  frontier: {count:4d} cells at map ({x:+.2f}, {y:+.2f}) m"
            )
        if not clusters:
            self.get_logger().info(
                "  no reachable frontiers: the room is mapped, or the rover is"
                " boxed in by obstacles"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapCoverageReporter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
