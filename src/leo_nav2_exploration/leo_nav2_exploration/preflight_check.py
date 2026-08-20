"""Read-only ROS graph, freshness, and TF audit for the overlay."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


PROFILES = {
    "sim_leo1": {
        "raw_scan": "/leo1/scan",
        "scan": "/leo1/scan_filtered",
        "odom": "/leo1/odom",
        "cloud": "/leo1/camera/points",
        "image": "/leo1/camera/image",
        "camera_info": "/leo1/camera/camera_info",
        "imu": "/leo1/imu/data_real",
        "base": "leo1/base_link",
        "odom_frame": "leo1/odom",
        "nav": "/leo1/cmd_vel_nav",
        "smoothed": "/leo1/cmd_vel_smoothed",
        "guarded": "/leo1/cmd_vel_guarded",
        "final": "/leo1/cmd_vel",
    },
    "real_root": {
        "raw_scan": "/scan",
        "scan": "/scan_filtered",
        "odom": "/wheel_odom",
        "cloud": "/camera/camera/depth/color/points",
        "image": "/camera/camera/color/image_raw",
        "camera_info": "/camera/camera/color/camera_info",
        "imu": "/imu/data",
        "base": "base_footprint",
        "odom_frame": "odom",
        "nav": "/cmd_vel_nav",
        "smoothed": "/cmd_vel_smoothed",
        "guarded": "/cmd_vel_guarded",
        "final": "/cmd_vel",
    },
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit the live overlay without publishing commands or TF.")
    parser.add_argument("--profile", choices=sorted(PROFILES), default="sim_leo1")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--freshness", type=float, default=1.5)
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument(
        "--require-imu",
        action="store_true",
        help="also require a fresh sensor_msgs/Imu (needed by the EKF).",
    )
    parser.add_argument(
        "--require-aruco",
        action="store_true",
        help="also require the RGB image and camera_info the ArUco detector reads.",
    )
    parser.add_argument(
        "--expect-ekf",
        action="store_true",
        help=(
            "the EKF is expected to own odom -> base. Requires a publisher on "
            "--ekf-odom-topic and FAILS if a second known odometry-TF publisher "
            "is also on /tf, which tf2 accepts silently while consumers see the "
            "pose flip between two estimates."
        ),
    )
    parser.add_argument("--ekf-odom-topic", default="/odometry/filtered")
    return parser


class PreflightNode(Node):
    def __init__(self, profile: dict, require_camera: bool,
                 require_imu: bool = False, require_aruco: bool = False) -> None:
        super().__init__("leo_nav2_preflight")
        self.profile = profile
        self.require_camera = require_camera
        self.require_imu = require_imu
        self.require_aruco = require_aruco
        self.imu_stamp: Optional[float] = None
        self.image_stamp: Optional[float] = None
        self.camera_info_stamp: Optional[float] = None
        self.raw_scan_stamp: Optional[float] = None
        self.scan_stamp: Optional[float] = None
        self.odom_stamp: Optional[float] = None
        self.map_stamp: Optional[float] = None
        self.cloud_stamp: Optional[float] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaserScan, profile["raw_scan"], self._raw_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, profile["scan"], self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, profile["odom"], self._odom, 20)
        self.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        if require_camera:
            self.create_subscription(PointCloud2, profile["cloud"], self._cloud, qos_profile_sensor_data)
        if require_imu:
            self.create_subscription(Imu, profile["imu"], self._imu, qos_profile_sensor_data)
        if require_aruco:
            self.create_subscription(Image, profile["image"], self._image, qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, profile["camera_info"], self._camera_info, qos_profile_sensor_data)

    @staticmethod
    def _has_valid_scan_data(message: LaserScan) -> bool:
        return any(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )

    def _raw_scan(self, message: LaserScan) -> None:
        if self._has_valid_scan_data(message):
            self.raw_scan_stamp = time.monotonic()

    def _scan(self, message: LaserScan) -> None:
        if self._has_valid_scan_data(message):
            self.scan_stamp = time.monotonic()

    def _odom(self, message: Odometry) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        if all(math.isfinite(value) for value in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.odom_stamp = time.monotonic()

    def _map(self, _message: OccupancyGrid) -> None:
        self.map_stamp = time.monotonic()

    def _cloud(self, message: PointCloud2) -> None:
        if message.width * message.height > 0:
            self.cloud_stamp = time.monotonic()

    def _imu(self, message: Imu) -> None:
        w = message.angular_velocity
        if all(math.isfinite(value) for value in (w.x, w.y, w.z)):
            self.imu_stamp = time.monotonic()

    def _image(self, message: Image) -> None:
        if message.width * message.height > 0:
            self.image_stamp = time.monotonic()

    def _camera_info(self, message: CameraInfo) -> None:
        # A zero focal length means the driver published a placeholder; solvePnP
        # would return nonsense rather than fail.
        if message.k[0] > 0.0:
            self.camera_info_stamp = time.monotonic()


def _endpoint_names(endpoints) -> list[str]:
    return sorted(
        f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}".replace("//", "/")
        for endpoint in endpoints
    )


def _check_transform(node: PreflightNode, target: str, source: str) -> tuple[bool, str]:
    try:
        node.tf_buffer.lookup_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=0.2),
        )
        return True, f"TF {target} <- {source} available"
    except TransformException as exc:
        return False, f"TF {target} <- {source} unavailable: {exc}"


def main(args=None) -> int:
    raw = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parser().parse_args(remove_ros_args(args=raw)[1:])
    if cli.timeout <= 0.0 or cli.freshness <= 0.0:
        print("ERROR: timeout and freshness must be positive", file=sys.stderr)
        return 2
    profile = PROFILES[cli.profile]
    rclpy.init(args=raw)
    node = PreflightNode(profile, cli.require_camera, cli.require_imu, cli.require_aruco)
    try:
        deadline = time.monotonic() + cli.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            ready = (
                node.raw_scan_stamp is not None
                and node.scan_stamp is not None
                and node.odom_stamp is not None
                and node.map_stamp is not None
            )
            if cli.require_camera:
                ready = ready and node.cloud_stamp is not None
            if cli.require_imu:
                ready = ready and node.imu_stamp is not None
            if cli.require_aruco:
                ready = ready and node.image_stamp is not None                     and node.camera_info_stamp is not None
            chain_ready = all(
                node.get_publishers_info_by_topic(profile[key])
                for key in ("nav", "smoothed", "guarded", "final")
            )
            if ready and chain_ready:
                break

        checks: list[tuple[bool, str]] = []
        now = time.monotonic()
        for label, stamp in (
            ("raw scan", node.raw_scan_stamp),
            ("filtered scan", node.scan_stamp),
            ("odometry", node.odom_stamp),
            ("map", node.map_stamp),
        ):
            ok = stamp is not None and now - stamp <= cli.freshness
            detail = "missing" if stamp is None else f"age={now - stamp:.3f}s"
            checks.append((ok, f"{label} freshness: {detail}"))
        optional = []
        if cli.require_camera:
            optional.append(("camera cloud", node.cloud_stamp))
        if cli.require_imu:
            optional.append(("imu", node.imu_stamp))
        if cli.require_aruco:
            optional.append(("camera image", node.image_stamp))
            optional.append(("camera_info", node.camera_info_stamp))
        for label, stamp in optional:
            ok = stamp is not None and now - stamp <= cli.freshness
            detail = "missing" if stamp is None else f"age={now - stamp:.3f}s"
            checks.append((ok, f"{label} freshness: {detail}"))

        raw_scan_publishers = node.get_publishers_info_by_topic(profile["raw_scan"])
        scan_publishers = node.get_publishers_info_by_topic(profile["scan"])
        odom_publishers = node.get_publishers_info_by_topic(profile["odom"])
        map_publishers = node.get_publishers_info_by_topic("/map")
        checks.append((len(raw_scan_publishers) >= 1, f"raw scan publishers={_endpoint_names(raw_scan_publishers)}"))
        checks.append((len(scan_publishers) == 1, f"filtered scan publishers={_endpoint_names(scan_publishers)}"))
        checks.append((len(odom_publishers) == 1, f"odometry publishers={_endpoint_names(odom_publishers)}"))
        checks.append((len(map_publishers) == 1, f"map publishers={_endpoint_names(map_publishers)}"))

        expected_counts = {"nav": (1, 2), "smoothed": (1, 1), "guarded": (1, 1), "final": (1, 1)}
        for key, (minimum, maximum) in expected_counts.items():
            endpoints = node.get_publishers_info_by_topic(profile[key])
            names = _endpoint_names(endpoints)
            ok = minimum <= len(endpoints) <= maximum
            if key == "final" and len(endpoints) == 1:
                ok = ok and "collision_monitor" in endpoints[0].node_name
            checks.append((ok, f"{key} command publishers on {profile[key]}={names}"))

        if cli.expect_ekf:
            # Exactly one node may publish odom -> base. tf2 does not complain
            # about two; the pose simply alternates between them, which reads
            # downstream as a SLAM fault.
            ekf_publishers = node.get_publishers_info_by_topic(cli.ekf_odom_topic)
            checks.append((
                len(ekf_publishers) >= 1,
                f"EKF output on {cli.ekf_odom_topic}={_endpoint_names(ekf_publishers)}",
            ))
            tf_publishers = _endpoint_names(node.get_publishers_info_by_topic("/tf"))
            conflicting = [name for name in tf_publishers
                           if "wheel_odom_tf" in name or "sim_realism_odom" in name]
            checks.append((
                not conflicting,
                f"no second odom TF publisher: /tf publishers={tf_publishers}"
                + (f" CONFLICT={conflicting}" if conflicting else ""),
            ))

        checks.append(_check_transform(node, profile["odom_frame"], profile["base"]))
        checks.append(_check_transform(node, "map", profile["base"]))

        failed = 0
        print(f"profile: {cli.profile}")
        for ok, message in checks:
            print(f"{'PASS' if ok else 'FAIL'}: {message}")
            failed += 0 if ok else 1
        if failed:
            print(f"Preflight failed: {failed} check(s)", file=sys.stderr)
            return 4
        print("Preflight passed. This verifies graph ownership and data freshness, not physical clearance.")
        return 0
    finally:
        del node.tf_listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
