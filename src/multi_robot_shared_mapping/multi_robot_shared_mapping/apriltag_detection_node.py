#!/usr/bin/env python3
"""
AprilTag detection for both Leo rovers using pupil_apriltags (tag36h11).

Subscribes to /leoN/camera/image and /leoN/camera/camera_info.
Publishes visualization_msgs/MarkerArray on /leoN/tag_detections with:
  - marker.id = AprilTag ID
  - marker.pose in {robot}/map frame (via TF camera optical frame -> map)

FRAME CONVENTION (important):
pupil_apriltags returns camera_T_tag in the camera OPTICAL convention
(z forward, x right, y down). The Gazebo sensor frame in the image header
(leoN/sensor_camera_link) uses the ROS body convention (x forward, z up).
This node therefore publishes a static TF:
  leoN/sensor_camera_link -> leoN/sensor_camera_optical_frame
with the standard link->optical rotation RPY(-pi/2, 0, -pi/2) and expresses
the tag pose in the optical frame before transforming to the map:
  map_T_tag = map_T_camera_optical * camera_optical_T_tag

If pupil_apriltags is missing, the node logs a clear error and does not publish
fake detections.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    _HAS_CV = True
except ImportError:
    cv2 = None
    np = None
    CvBridge = None
    _HAS_CV = False

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector

    _HAS_PUPIL = True
except ImportError:
    PupilAprilTagDetector = None
    _HAS_PUPIL = False

try:
    import tf2_geometry_msgs

    _HAS_TF2_GEOM = True
except ImportError:
    tf2_geometry_msgs = None
    _HAS_TF2_GEOM = False


def rotation_matrix_to_quaternion(rot: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion x,y,z,w."""
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class AprilTagDetectionNode(Node):
    ROBOTS = ("leo1", "leo2")

    def __init__(self):
        super().__init__("apriltag_detection_node")

        self.declare_parameter("tag_size_m", 0.35)
        self.declare_parameter("log_every_n_frames", 30)

        self.frame_counts: Dict[str, int] = {robot: 0 for robot in self.ROBOTS}
        self.logged_encoding: Dict[str, bool] = {robot: False for robot in self.ROBOTS}
        self.camera_info: Dict[str, Optional[CameraInfo]] = {
            robot: None for robot in self.ROBOTS
        }
        self.bridge = CvBridge() if _HAS_CV else None
        self.detector = None
        self.detector_ready = False
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self._init_detector()
        self._publish_optical_frames()

        self.publishers_map: Dict[str, rclpy.publisher.Publisher] = {}
        for robot in self.ROBOTS:
            self.create_subscription(
                CameraInfo,
                f"/{robot}/camera/camera_info",
                lambda msg, name=robot: self.camera_info_cb(name, msg),
                10,
            )
            self.create_subscription(
                Image,
                f"/{robot}/camera/image",
                lambda msg, name=robot: self.image_cb(name, msg),
                10,
            )
            self.publishers_map[robot] = self.create_publisher(
                MarkerArray,
                f"/{robot}/tag_detections",
                10,
            )

        if self.detector_ready:
            self.get_logger().info(
                "AprilTag detection node started (pupil_apriltags tag36h11)"
            )
        else:
            self.get_logger().error(
                "AprilTag detection node started but detection is DISABLED"
            )

    def _init_detector(self):
        if not _HAS_CV:
            self.get_logger().error(
                "OpenCV/cv_bridge not available; AprilTag detection disabled"
            )
            return

        if not _HAS_PUPIL:
            self.get_logger().error(
                "pupil_apriltags not installed; AprilTag detection disabled"
            )
            return

        if not _HAS_TF2_GEOM:
            self.get_logger().error(
                "tf2_geometry_msgs not available; AprilTag detection disabled"
            )
            return

        self.detector = PupilAprilTagDetector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self.detector_ready = True
        self.get_logger().info("Using pupil_apriltags detector (tag36h11)")

    def _publish_optical_frames(self):
        """
        Static TF: leoN/sensor_camera_link -> leoN/sensor_camera_optical_frame.

        pupil_apriltags poses follow the optical convention (z forward,
        x right, y down); the sensor link uses the ROS body convention
        (x forward, z up). Rotation is the standard RPY(-pi/2, 0, -pi/2),
        i.e. quaternion (-0.5, 0.5, -0.5, 0.5).
        """
        transforms = []
        for robot in self.ROBOTS:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = f"{robot}/sensor_camera_link"
            tf_msg.child_frame_id = f"{robot}/sensor_camera_optical_frame"
            tf_msg.transform.rotation.x = -0.5
            tf_msg.transform.rotation.y = 0.5
            tf_msg.transform.rotation.z = -0.5
            tf_msg.transform.rotation.w = 0.5
            transforms.append(tf_msg)
        self.static_tf_broadcaster.sendTransform(transforms)

    def _optical_frame(self, robot_name: str) -> str:
        return f"{robot_name}/sensor_camera_optical_frame"

    def _robot_pose_xy(self, robot_name: str) -> Optional[Tuple[float, float]]:
        """Robot base position in its own map frame (debug logging only)."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                f"{robot_name}/map",
                f"{robot_name}/base_footprint",
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except tf2_ros.TransformException:
            return None
        return (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
        )

    def camera_info_cb(self, robot_name: str, msg: CameraInfo):
        self.camera_info[robot_name] = msg

    def _image_to_gray(self, msg: Image) -> Optional[np.ndarray]:
        encoding = (msg.encoding or "").lower()

        if encoding in ("mono8", "8uc1"):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        if encoding in ("rgb8",):
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        if encoding in ("rgba8",):
            rgba = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgba8")
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2GRAY)

        if encoding in ("bgr8",):
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        if encoding in ("bgra8",):
            bgra = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2GRAY)

        # Gazebo / ros_gz_bridge may use other encodings; try passthrough.
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if len(raw.shape) == 2:
                return raw
            if raw.shape[2] == 3:
                return cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
            if raw.shape[2] == 4:
                return cv2.cvtColor(raw, cv2.COLOR_BGRA2GRAY)
        except Exception as exc:
            self.get_logger().warn(
                f"{msg.header.frame_id}: unsupported encoding '{msg.encoding}': {exc}"
            )
        return None

    def _camera_params(self, robot_name: str) -> Optional[Tuple[float, float, float, float]]:
        info = self.camera_info.get(robot_name)
        if info is None or len(info.k) < 9:
            return None
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        return fx, fy, cx, cy

    def _pose_to_map(
        self,
        robot_name: str,
        stamp,
        camera_frame: str,
        rot: np.ndarray,
        trans: np.ndarray,
    ) -> Optional[PoseStamped]:
        pose_cam = PoseStamped()
        pose_cam.header.stamp = stamp
        pose_cam.header.frame_id = camera_frame
        pose_cam.pose.position.x = float(trans[0])
        pose_cam.pose.position.y = float(trans[1])
        pose_cam.pose.position.z = float(trans[2])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot)
        pose_cam.pose.orientation.x = qx
        pose_cam.pose.orientation.y = qy
        pose_cam.pose.orientation.z = qz
        pose_cam.pose.orientation.w = qw

        map_frame = f"{robot_name}/map"
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                camera_frame,
                Time.from_msg(stamp) if stamp.sec or stamp.nanosec else Time(),
                timeout=Duration(seconds=0.2),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"{robot_name}: TF {camera_frame} -> {map_frame} unavailable: {exc}"
            )
            return None

        pose_map = PoseStamped()
        pose_map.header.stamp = stamp
        pose_map.header.frame_id = map_frame
        pose_map.pose = tf2_geometry_msgs.do_transform_pose(pose_cam.pose, transform)
        return pose_map

    def image_cb(self, robot_name: str, msg: Image):
        self.frame_counts[robot_name] += 1
        log_every = int(self.get_parameter("log_every_n_frames").value)

        if not self.logged_encoding[robot_name]:
            self.logged_encoding[robot_name] = True
            cam_frame = msg.header.frame_id or f"{robot_name}/sensor_camera_link"
            self.get_logger().info(
                f"{robot_name}: first image encoding='{msg.encoding}' "
                f"frame_id='{cam_frame}' size={msg.width}x{msg.height}"
            )

        if self.frame_counts[robot_name] % log_every == 0:
            info = self.camera_info.get(robot_name)
            self.get_logger().info(
                f"{robot_name}: received image #{self.frame_counts[robot_name]} "
                f"encoding={msg.encoding} camera_info={'yes' if info else 'no'}"
            )

        if not self.detector_ready:
            return

        gray = self._image_to_gray(msg)
        if gray is None:
            return

        camera_params = self._camera_params(robot_name)
        tag_size_m = float(self.get_parameter("tag_size_m").value)
        # pupil_apriltags poses are in the optical convention, so TF lookups
        # must use the optical frame, NOT the header's sensor link frame.
        camera_frame = self._optical_frame(robot_name)
        debug_frame = self.frame_counts[robot_name] % log_every == 0

        detect_kwargs = {}
        if camera_params is not None:
            detect_kwargs["estimate_tag_pose"] = True
            detect_kwargs["camera_params"] = camera_params
            detect_kwargs["tag_size"] = tag_size_m

        try:
            detections = self.detector.detect(gray, **detect_kwargs)
        except Exception as exc:
            self.get_logger().error(f"{robot_name}: detection failed: {exc}")
            return

        markers = MarkerArray()
        detected_ids: List[int] = []

        for det in detections:
            tag_id = int(det.tag_id)
            detected_ids.append(tag_id)

            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.ns = f"{robot_name}_apriltag"
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.02
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.9

            if camera_params is not None and det.pose_t is not None and det.pose_R is not None:
                pose_map = self._pose_to_map(
                    robot_name,
                    msg.header.stamp,
                    camera_frame,
                    det.pose_R,
                    det.pose_t,
                )
                if pose_map is None:
                    continue
                marker.header.frame_id = pose_map.header.frame_id
                marker.pose = pose_map.pose

                if debug_frame:
                    raw = det.pose_t.flatten()
                    robot_xy = self._robot_pose_xy(robot_name)
                    robot_text = (
                        f"({robot_xy[0]:.2f}, {robot_xy[1]:.2f})" if robot_xy else "n/a"
                    )
                    distance = math.sqrt(float(raw[0]) ** 2 + float(raw[1]) ** 2 + float(raw[2]) ** 2)
                    self.get_logger().info(
                        f"{robot_name} tag_{tag_id} DEBUG | "
                        f"image_frame='{msg.header.frame_id}' pose_frame='{camera_frame}' | "
                        f"camera_T_tag=({raw[0]:.2f}, {raw[1]:.2f}, {raw[2]:.2f}) "
                        f"dist={distance:.2f}m | "
                        f"map_T_tag=({pose_map.pose.position.x:.2f}, "
                        f"{pose_map.pose.position.y:.2f}, {pose_map.pose.position.z:.2f}) | "
                        f"robot_xy={robot_text}"
                    )
            else:
                # Fallback: pixel center in camera frame (aligner needs map frame ideally).
                cx_px = float(det.center[0])
                cy_px = float(det.center[1])
                marker.header.frame_id = camera_frame
                marker.pose.position.x = cx_px * 0.001
                marker.pose.position.y = cy_px * 0.001
                marker.pose.position.z = tag_size_m
                marker.pose.orientation.w = 1.0

            markers.markers.append(marker)

        if detected_ids:
            self.get_logger().info(
                f"{robot_name}: detected {len(detected_ids)} tag(s) "
                f"IDs={sorted(detected_ids)} frame_id={camera_frame}"
            )
        elif self.frame_counts[robot_name] % log_every == 0:
            self.get_logger().info(f"{robot_name}: detected 0 tags")

        self.publishers_map[robot_name].publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
