"""Pose recovery for the ArUco detector, checked against synthetic views.

Renders a marker at a *known* pose with known intrinsics, then runs the
detector's own detection and pose code over the rendered image. The point is
to catch the errors that hardware would otherwise catch for us:

* a `marker_length` that does not mean the black square,
* the optical -> body rotation applied the wrong way round (which silently
  rotates every detection by 90 degrees rather than failing),
* an OpenCV version whose `cv2.aruco` entry points have moved again.

No ROS, no simulator; `python3 -m pytest test/test_aruco_pose.py` is enough.
"""

import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODULE = os.path.join(_HERE, '..', 'leo_nav2_exploration', 'aruco_detector.py')


def _load_detector_module():
    """Import aruco_detector.py without importing the ROS package."""
    spec = importlib.util.spec_from_file_location('aruco_detector_under_test',
                                                  os.path.abspath(_MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    ad = _load_detector_module()
except Exception as exc:  # pragma: no cover - missing rclpy etc.
    ad = None
    _IMPORT_ERROR = exc

pytestmark = pytest.mark.skipif(ad is None, reason='aruco_detector import failed')

WIDTH, HEIGHT = 640, 480
FX = FY = 554.4        # the sim camera: 640 px across a 60 degree horizontal FOV
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
D = np.zeros((1, 5))
MARKER_ID = 7
MARKER_LENGTH = 0.20


def _marker_image(px=400):
    dict_id = cv2.aruco.DICT_4X4_50
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    else:  # OpenCV < 4.7
        dictionary = cv2.aruco.Dictionary_get(dict_id)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, MARKER_ID, px)
    return cv2.aruco.drawMarker(dictionary, MARKER_ID, px)


def _render(position, yaw_deg=0.0, length=MARKER_LENGTH):
    """A camera view of the marker centred at `position` in the OPTICAL frame.

    `position` is (x right, y down, z forward) metres. The marker faces the
    camera, rotated by `yaw_deg` about the optical y axis.
    """
    half = length / 2.0
    # Same corner order the detector assumes: TL, TR, BR, BL.
    obj = np.array([[-half, half, 0.0], [half, half, 0.0],
                    [half, -half, 0.0], [-half, -half, 0.0]])
    a = math.radians(yaw_deg)
    # Rotation about the optical y (down) axis.
    R = np.array([[math.cos(a), 0.0, math.sin(a)],
                  [0.0, 1.0, 0.0],
                  [-math.sin(a), 0.0, math.cos(a)]])
    # The marker plane's y runs "up" in marker coordinates and "down" in the
    # optical frame, so flip it before placing the corners.
    flip = np.diag([1.0, -1.0, 1.0])
    pts_cam = (R @ flip @ obj.T).T + np.asarray(position, dtype=np.float64)

    # Supersample: warping a hard-edged marker straight into a 640x480 grid
    # aliases its corners by most of a pixel, and at 4 m the marker is only
    # ~28 px across, so that aliasing alone shows up as centimetres of range
    # error. A real sensor integrates over each pixel; rendering at 4x and
    # downsampling with INTER_AREA models that.
    ss = 4
    Kss = K.copy()
    Kss[:2, :] *= ss
    image_pts, _ = cv2.projectPoints(pts_cam, np.zeros(3), np.zeros(3), Kss, D)
    image_pts = image_pts.reshape(4, 2).astype(np.float32)

    marker = _marker_image()
    n = marker.shape[0]
    src = np.array([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]],
                   dtype=np.float32)
    warp = cv2.getPerspectiveTransform(src, image_pts)
    # White page behind the marker: without a quiet zone `detectMarkers` cannot
    # find the quad at all, which is exactly why the sim world grew a backing
    # board rather than relying on a padded texture.
    canvas = np.full((HEIGHT * ss, WIDTH * ss), 255, dtype=np.uint8)
    big = cv2.warpPerspective(marker, warp, (WIDTH * ss, HEIGHT * ss),
                              borderMode=cv2.BORDER_TRANSPARENT,
                              dst=canvas)
    rendered = cv2.resize(big, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(rendered, cv2.COLOR_GRAY2BGR), pts_cam


def _detect_pose(bgr, length=MARKER_LENGTH):
    detect, _ = ad._make_detector('DICT_4X4_50')
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)
    assert ids is not None and len(ids) == 1, 'marker not detected'
    assert int(ids.flatten()[0]) == MARKER_ID
    half = length / 2.0
    obj = np.array([[-half, half, 0.0], [half, half, 0.0],
                    [half, -half, 0.0], [-half, -half, 0.0]])
    ok, rvec, tvec = cv2.solvePnP(obj, corners[0].reshape(4, 2).astype(np.float64),
                                  K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok
    return tvec.reshape(3), rvec


@pytest.mark.parametrize('distance', [1.0, 2.0, 4.0])
def test_range_is_recovered(distance):
    bgr, _ = _render((0.0, 0.0, distance))
    t, _ = _detect_pose(bgr)
    # Depth error from corner noise grows as Z^2 * delta / (f * L). With half a
    # pixel of corner uncertainty that is 3.6 mm at 1 m and 58 mm at 4 m, so a
    # flat percentage tolerance would either pass everything or fail the far
    # case for the wrong reason.
    budget = 0.5 * distance ** 2 / (FX * MARKER_LENGTH) + 0.01
    assert abs(t[2] - distance) < budget, (
        f'range off by {abs(t[2] - distance):.3f} m at {distance} m '
        f'(budget {budget:.3f} m)')
    assert abs(t[0]) < 0.02 and abs(t[1]) < 0.02


def test_off_axis_position_is_recovered():
    bgr, _ = _render((0.35, -0.20, 2.0))
    t, _ = _detect_pose(bgr)
    assert t[0] == pytest.approx(0.35, abs=0.03)
    assert t[1] == pytest.approx(-0.20, abs=0.03)
    assert t[2] == pytest.approx(2.0, rel=0.03)


def test_wrong_marker_length_scales_range_proportionally():
    """The failure mode the deployment guide warns about, made explicit."""
    bgr, _ = _render((0.0, 0.0, 3.0), length=0.20)
    t_right, _ = _detect_pose(bgr, length=0.20)
    t_wrong, _ = _detect_pose(bgr, length=0.15)
    assert t_right[2] == pytest.approx(3.0, rel=0.02)
    # Telling solvePnP the marker is 3/4 its true size puts it 3/4 as far away.
    assert t_wrong[2] == pytest.approx(3.0 * 0.15 / 0.20, rel=0.02)


def test_optical_to_body_maps_straight_ahead_to_forward():
    """Straight ahead in the optical frame must become +x in the body frame."""
    forward = ad._OPTICAL_TO_BODY @ np.array([0.0, 0.0, 1.0])
    assert forward == pytest.approx([1.0, 0.0, 0.0])
    right = ad._OPTICAL_TO_BODY @ np.array([1.0, 0.0, 0.0])
    assert right == pytest.approx([0.0, -1.0, 0.0])       # right is -y
    down = ad._OPTICAL_TO_BODY @ np.array([0.0, 1.0, 0.0])
    assert down == pytest.approx([0.0, 0.0, -1.0])        # down is -z
    assert np.linalg.det(ad._OPTICAL_TO_BODY) == pytest.approx(1.0)


def test_a_marker_2_m_ahead_lands_2_m_in_front_in_the_body_frame():
    bgr, _ = _render((0.0, 0.0, 2.0))
    t, _ = _detect_pose(bgr)
    body = ad._OPTICAL_TO_BODY @ t
    assert body[0] == pytest.approx(2.0, rel=0.03)
    assert abs(body[1]) < 0.03
    assert abs(body[2]) < 0.03


def test_quaternion_from_matrix_round_trips():
    for angles in [(0.0, 0.0, 0.0), (0.3, -0.2, 1.1), (math.pi / 2, 0.0, 0.0),
                   (0.0, math.pi / 2, 0.0), (2.9, 0.1, -3.0)]:
        R = _rotation(*angles)
        q = np.array(ad._quat_from_matrix(R))
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
        assert _matrix_from_quat(q) == pytest.approx(R, abs=1e-6)


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _matrix_from_quat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
