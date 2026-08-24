"""Real ArUco marker detector (OpenCV) for the Leo Rover.

Replaces `leo_rover_exploration.mock_aruco_detector` behind the same topic
contract: a `visualization_msgs/MarkerArray` where each `Marker` carries the
ArUco id in `.id` and the marker pose in the map frame, so the explorer's
registry needs no change.

Pipeline
    Image + CameraInfo -> cv2.aruco.detectMarkers -> solvePnP(IPPE_SQUARE)
    -> pose in the camera optical frame -> tf2 into the map frame
    -> gating (range, reprojection error, repeat-sighting confirmation)
    -> range-weighted average registry -> MarkerArray (+ TF, debug image)

Two things this node is deliberately careful about, because both are silent
failure modes on hardware:

*Optical vs body frame.* `solvePnP` returns a pose in the OpenCV optical
convention (z forward, x right, y down). A RealSense publishes images stamped
with `*_optical_frame`, which already has that convention, so the pose can be
transformed directly. Gazebo's `rgbd_camera` stamps images with the *link*
frame (x forward, ROS body convention), so the optical->body rotation has to
be applied first. Getting this wrong yields detections that are silently
rotated by 90 degrees rather than an error, so `frame_is_optical` is an
explicit parameter and the resolved value is logged at startup.

*OpenCV API drift.* `cv2.aruco` was restructured in 4.7 and
`estimatePoseSingleMarkers` was later removed. Detection is done through
whichever API is present, and the pose always through plain `cv2.solvePnP`,
which has been stable across every version.
"""

import json
import math
import os
import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

# Optical (z fwd, x right, y down) expressed in body axes (x fwd, y left, z up).
# Columns are the optical x, y, z axes written in body coordinates.
_OPTICAL_TO_BODY = np.array(
    [[0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]],
    dtype=np.float64,
)

_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_APRILTAG_36h11': getattr(cv2.aruco, 'DICT_APRILTAG_36h11', cv2.aruco.DICT_4X4_50),
}


def _make_detector(dict_name):
    """Return (detect_fn, dictionary) working on both old and new cv2.aruco."""
    dict_id = _DICTS[dict_name]
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        # Corner refinement is what turns a ~2 px corner estimate into a
        # sub-pixel one; without it the pose of a small distant marker is noisy
        # enough to be useless for anything but presence detection.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return (lambda gray: detector.detectMarkers(gray)), dictionary
    dictionary = cv2.aruco.Dictionary_get(dict_id)   # OpenCV < 4.7
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return (lambda gray: cv2.aruco.detectMarkers(gray, dictionary,
                                                 parameters=params)), dictionary


def _draw_markers(img, corners, ids):
    try:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
    except cv2.error:
        pass
    return img


def _image_to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray, without depending on cv_bridge."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img.copy()
    if enc in ('mono8', '8uc1'):
        gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(msg.height, msg.step // 4, 4)[:, :msg.width, :]
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    raise ValueError(f'unsupported image encoding {msg.encoding}')


def _quat_from_matrix(R):
    """Rotation matrix -> (x, y, z, w), branch-stable form."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


class _Track:
    """Per-id detection history and range-weighted average map pose.

    The average is weighted by 1/range^2, not uniform. Depth error from
    sub-pixel corner noise grows with the square of the range, so a sighting at
    1 m is worth roughly twenty-five at 5 m; a uniform mean lets a crowd of
    distant, noisy frames outvote the few good close ones.
    """

    def __init__(self):
        self.hits = 0
        self.confirmed = False
        self.pos = np.zeros(3)
        self.wsum = 0.0
        self.quat = None
        self.last_seen = 0.0
        self.last_range = 0.0
        # The closest observation is the most accurate one: depth error from
        # corner noise grows with range squared. Keeping it separately from the
        # running mean gives a clean number to check extrinsics against.
        self.best_range = float('inf')
        self.best_pos = None


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        p = self.declare_parameter
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('detection_topic', '/aruco_detections')
        p('markers_topic', '/aruco_markers')
        p('debug_image_topic', '/aruco/debug_image')
        p('publish_debug_image', False)
        p('map_frame', 'map')
        p('dictionary', 'DICT_4X4_50')
        p('marker_length', 0.15)          # side of the black square, metres
        p('max_range', 6.0)               # beyond this the pose is not trusted
        p('min_marker_px', 18.0)          # smaller than this -> ignore
        p('max_reprojection_error_px', 4.0)
        p('min_hits', 3)                  # sightings before a marker is trusted
        # Ids you actually put on the wall. Anything else is rejected outright.
        # DICT_4X4_50 has weak error correction and office scenes are full of
        # high-contrast rectangles; a run of this detector in simulation threw
        # up a spurious id 25 with no marker 25 anywhere in the world. The
        # min_hits gate suppressed it, but an id allowlist costs nothing and
        # does not depend on the false detection failing to repeat.
        # Empty list = accept every id in the dictionary.
        p('allowed_ids', [1, 2, 3, 4, 5, 6, 7, 8])
        p('publish_tf', True)
        # Two rovers each running a detector would both broadcast `aruco_<id>`
        # under different parents, which gives that frame two parents and
        # corrupts the whole tree. Prefix it per robot (e.g. "leo1/").
        p('tag_frame_prefix', '')
        p('frame_is_optical', True)       # RealSense: true. Gazebo link: false.
        p('camera_frame_override', '')    # non-empty replaces header.frame_id
        p('tf_timeout', 0.2)
        p('rate_limit_hz', 0.0)           # 0 = process every frame
        # Offline scoring hook: the confirmed registry, rewritten periodically,
        # so a headless run can be graded after the fact without a bag.
        p('registry_file', '')
        # Per-detection CSV. `marker_length` is the one number a deployment can
        # get wrong without anything erroring -- the pose simply lands short or
        # long along the view ray -- so the raw observations are kept for an
        # after-the-fact check against known marker positions.
        p('samples_file', '')

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.marker_length = float(g('marker_length'))
        self.max_range = float(g('max_range'))
        self.min_marker_px = float(g('min_marker_px'))
        self.max_reproj = float(g('max_reprojection_error_px'))
        self.min_hits = int(g('min_hits'))
        self.allowed_ids = set(int(v) for v in (g('allowed_ids') or []))
        self.publish_tf = bool(g('publish_tf'))
        self.tag_frame_prefix = str(g('tag_frame_prefix'))
        self.frame_is_optical = bool(g('frame_is_optical'))
        self.frame_override = g('camera_frame_override')
        self.tf_timeout = float(g('tf_timeout'))
        rate = float(g('rate_limit_hz'))
        self.min_period = (1.0 / rate) if rate > 0.0 else 0.0
        self.publish_debug = bool(g('publish_debug_image'))

        self.detect, _ = _make_detector(g('dictionary'))
        # Marker corner order from detectMarkers is TL, TR, BR, BL in the
        # marker's own plane (z out of the marker face).
        h = self.marker_length / 2.0
        self.obj_pts = np.array([[-h, h, 0.0], [h, h, 0.0],
                                 [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)

        self.K = None
        self.D = None
        self.tracks = {}
        self.lock = threading.Lock()
        self.last_proc = 0.0
        self.frames = 0
        self.det_frames = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        sensor_qos = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(MarkerArray, g('detection_topic'), 10)
        self.pub_all = self.create_publisher(MarkerArray, g('markers_topic'), 10)
        self.pub_poses = self.create_publisher(PoseArray, g('markers_topic') + '_poses', 10)
        self.pub_dbg = (self.create_publisher(Image, g('debug_image_topic'), 1)
                        if self.publish_debug else None)

        self.create_subscription(CameraInfo, g('camera_info_topic'),
                                 self.on_info, sensor_qos)
        self.create_subscription(Image, g('image_topic'), self.on_image, sensor_qos)
        self.registry_file = g('registry_file')
        self.samples_file = g('samples_file')
        if self.samples_file:
            with open(self.samples_file, 'w') as fh:
                fh.write('t,id,range_m,side_px,reproj_px,'
                         'cam_x,cam_y,cam_z,map_x,map_y,map_z\n')
        self.create_timer(10.0, self.report)
        if self.registry_file:
            self.create_timer(5.0, self.dump_registry)

        self.get_logger().info(
            f"aruco_detector: dict={g('dictionary')} len={self.marker_length} m "
            f"image={g('image_topic')} optical_frame={self.frame_is_optical} "
            f"map_frame={self.map_frame} cv2={cv2.__version__} "
            f"allowed_ids={sorted(self.allowed_ids) or 'any'}")

    # ---------------------------------------------------------------- inputs

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(1, -1)
            if self.D.size == 0:
                self.D = np.zeros((1, 5))
            self.get_logger().info(
                f'camera intrinsics: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f} '
                f'distortion={self.D.size} coeffs')

    def on_image(self, msg):
        if self.K is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.min_period and (now - self.last_proc) < self.min_period:
            return
        self.last_proc = now
        self.frames += 1

        try:
            bgr = _image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        if ids is None or len(ids) == 0:
            if self.pub_dbg is not None:
                self.publish_debug_image(msg, bgr)
            return
        self.det_frames += 1

        cam_frame = self.frame_override or msg.header.frame_id
        detections = MarkerArray()
        for corner, mid in zip(corners, ids.flatten()):
            det = self.pose_of(corner, int(mid), cam_frame, msg.header.stamp)
            if det is not None:
                detections.markers.append(det)

        if detections.markers:
            self.pub.publish(detections)
            self.publish_registry(msg.header.stamp)
        if self.pub_dbg is not None:
            self.publish_debug_image(msg, _draw_markers(bgr, corners, ids))

    # ------------------------------------------------------------------ pose

    def pose_of(self, corner, mid, cam_frame, stamp):
        if self.allowed_ids and mid not in self.allowed_ids:
            self.get_logger().warn(
                f'ignoring ArUco id {mid}: not in allowed_ids '
                f'{sorted(self.allowed_ids)}', throttle_duration_sec=30.0)
            return None
        pts = corner.reshape(4, 2).astype(np.float64)
        side_px = max(np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4))
        if side_px < self.min_marker_px:
            return None

        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, pts, self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None

        proj, _ = cv2.projectPoints(self.obj_pts, rvec, tvec, self.K, self.D)
        reproj = float(np.sqrt(((proj.reshape(4, 2) - pts) ** 2).sum(axis=1).mean()))
        if reproj > self.max_reproj:
            self.get_logger().debug(
                f'marker {mid} rejected: reprojection {reproj:.2f} px')
            return None

        rng = float(np.linalg.norm(tvec))
        if rng > self.max_range or rng < 0.05:
            return None

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3)
        if not self.frame_is_optical:
            # Camera frame is a ROS body frame; rotate the optical-convention
            # pose into it before handing it to tf2.
            t = _OPTICAL_TO_BODY @ t
            R = _OPTICAL_TO_BODY @ R

        map_pos, map_quat = self.to_map(t, R, cam_frame, stamp)
        if map_pos is None:
            return None

        if self.samples_file:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            with open(self.samples_file, 'a') as fh:
                fh.write(f'{now_s:.3f},{mid},{rng:.4f},{side_px:.2f},'
                         f'{reproj:.3f},{t[0]:.4f},{t[1]:.4f},{t[2]:.4f},'
                         f'{map_pos[0]:.4f},{map_pos[1]:.4f},{map_pos[2]:.4f}\n')

        with self.lock:
            tr = self.tracks.setdefault(mid, _Track())
            tr.hits += 1
            # A wall marker is static, so averaging across frames beats
            # single-frame corner noise -- but only if near sightings dominate.
            w = 1.0 / max(rng * rng, 1e-3)
            tr.wsum += w
            tr.pos += (map_pos - tr.pos) * (w / tr.wsum)
            tr.quat = map_quat
            tr.last_seen = self.get_clock().now().nanoseconds * 1e-9
            tr.last_range = rng
            if rng < tr.best_range:
                tr.best_range = rng
                tr.best_pos = map_pos.copy()
            if tr.hits >= self.min_hits and not tr.confirmed:
                tr.confirmed = True
                self.get_logger().info(
                    f'ArUco {mid} CONFIRMED at map '
                    f'({tr.pos[0]:.2f}, {tr.pos[1]:.2f}, {tr.pos[2]:.2f}) '
                    f'range {rng:.2f} m, reproj {reproj:.2f} px')
            confirmed = tr.confirmed
            pos = tr.pos.copy()

        if not confirmed:
            return None
        return self.marker_msg(mid, pos, map_quat, stamp)

    def to_map(self, t, R, cam_frame, stamp):
        """Camera-frame pose -> map frame. Returns (None, None) if TF is missing."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, cam_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException, tf2_ros.TransformException) as exc:
            self.get_logger().warn(
                f'no TF {self.map_frame} <- {cam_frame}: {exc}',
                throttle_duration_sec=5.0)
            return None, None

        q = tf.transform.rotation
        Rm = self.quat_to_matrix(q.x, q.y, q.z, q.w)
        p = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        return Rm @ t + p, _quat_from_matrix(Rm @ R)

    @staticmethod
    def quat_to_matrix(x, y, z, w):
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n == 0.0:
            return np.eye(3)
        x, y, z, w = x / n, y / n, z / n, w / n
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    # --------------------------------------------------------------- outputs

    def marker_msg(self, mid, pos, quat, stamp):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = 'aruco'
        m.id = int(mid)
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        if quat is not None:
            m.pose.orientation.x, m.pose.orientation.y = float(quat[0]), float(quat[1])
            m.pose.orientation.z, m.pose.orientation.w = float(quat[2]), float(quat[3])
        else:
            m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.marker_length
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.8
        return m

    def publish_registry(self, stamp):
        arr = MarkerArray()
        poses = PoseArray()
        poses.header.frame_id = self.map_frame
        poses.header.stamp = stamp
        with self.lock:
            items = [(mid, tr.pos.copy(), tr.quat)
                     for mid, tr in self.tracks.items() if tr.confirmed]
        for mid, pos, quat in items:
            m = self.marker_msg(mid, pos, quat, stamp)
            arr.markers.append(m)
            pose = Pose()
            pose.position = m.pose.position
            pose.orientation = m.pose.orientation
            poses.poses.append(pose)
            if self.tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header.frame_id = self.map_frame
                tf.header.stamp = stamp
                tf.child_frame_id = f'{self.tag_frame_prefix}aruco_{mid}'
                tf.transform.translation.x = m.pose.position.x
                tf.transform.translation.y = m.pose.position.y
                tf.transform.translation.z = m.pose.position.z
                tf.transform.rotation = m.pose.orientation
                self.tf_broadcaster.sendTransform(tf)
        if arr.markers:
            self.pub_all.publish(arr)
            self.pub_poses.publish(poses)

    def publish_debug_image(self, src, bgr):
        out = Image()
        out.header = src.header
        out.height, out.width = bgr.shape[0], bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.pub_dbg.publish(out)

    def dump_registry(self):
        with self.lock:
            data = {
                'frames': self.frames,
                'frames_with_detections': self.det_frames,
                'markers': [
                    {'id': int(mid), 'x': float(t.pos[0]), 'y': float(t.pos[1]),
                     'z': float(t.pos[2]), 'hits': t.hits,
                     'last_range_m': t.last_range,
                     'best_range_m': (None if t.best_range == float('inf')
                                      else t.best_range),
                     'best_x': (None if t.best_pos is None else float(t.best_pos[0])),
                     'best_y': (None if t.best_pos is None else float(t.best_pos[1])),
                     'best_z': (None if t.best_pos is None else float(t.best_pos[2])),
                     'quat': [float(v) for v in t.quat] if t.quat else None}
                    for mid, t in sorted(self.tracks.items()) if t.confirmed
                ],
                'pending': [int(m) for m, t in sorted(self.tracks.items())
                            if not t.confirmed],
            }
        tmp = self.registry_file + '.tmp'
        with open(tmp, 'w') as fh:
            json.dump(data, fh, indent=2)
        os.replace(tmp, self.registry_file)

    def report(self):
        with self.lock:
            confirmed = sorted(m for m, t in self.tracks.items() if t.confirmed)
            pending = sorted(m for m, t in self.tracks.items() if not t.confirmed)
        self.get_logger().info(
            f'frames={self.frames} with_detections={self.det_frames} '
            f'confirmed={confirmed} pending={pending}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
