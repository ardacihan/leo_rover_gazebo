#!/usr/bin/env python3
"""Throttle the RealSense streams into the topics the bag and the replay want.

Publishes, at a rate you choose:

    /bag/color/compressed        the driver's OWN jpeg, passed through
                                 untouched. ~157 kB/frame at 640x480.
    /bag/color/camera_info       colour intrinsics at 1 Hz. Tiny, and without
                                 it ArUco CANNOT be run offline: the detector
                                 needs K and D to solvePnP a marker pose.
    /bag/depth/compressed        aligned depth, PNG-encoded 16UC1 in mm.
                                 ~190 kB/frame. Only with DEPTH=1.
    /rob_4/camera/depth/camera_info   throttled copy of the depth intrinsics.

Those names are not arbitrary: `scripts/drive_replay/` reads exactly them
(`extract_bag.py`, `depth_to_points.py`, `probe_bag.py`), and that pipeline is
what reconstructs costmaps, Nav2 plans and frontier goals from a bag afterwards.
A bag recorded under any other name replays as pictures only.

RATE IS DECIMATION, NOT RESAMPLING. This keeps a frame when one period has
elapsed and drops the rest, so the achieved rate is always the camera's rate
divided by a whole number. Ask for 10 Hz from a 15 fps camera and you get 7.5,
not 10 -- it cannot invent the frames in between. Choose a camera fps that is a
whole multiple of what you want:

    want 10 Hz  ->  run the camera at 30 fps   (30/3)
    want  5 Hz  ->  15 fps  (15/3)  or 30 fps  (30/6)
    want  2 Hz  ->   6 fps  (6/3)   or 30 fps  (30/15)

The node measures what it actually achieved and warns if it is more than 20%
under what you asked for, because a silent shortfall here is a bag with a
quarter of the frames you planned for.

Sizes, from the 157 kB/frame measured at 640x480 and scaled by pixel count:

    profile      kB/frame   @2 Hz      @5 Hz       @10 Hz
    640x480         157     19 MB/min  46 MB/min    92 MB/min
    848x480        ~208     24         61          122
    1280x720       ~470     55        138          275

Depth is ~190 kB/frame at 640x480, ~570 kB at 1280x720, and feeds only the
offline costmap layer -- keep it slow. At 0.1 m/s, 2 Hz is a depth frame every
5 cm, and drive_replay throttles to ~5 Hz internally anyway.

Never bag the raw image topics. That is not a size argument, it is a stability
one: bagging raw 640x480 colour+depth pushed load average to 9.3 on 6 cores and
opened 0.4-0.5 s scan arrival gaps that tripped the explorer's watchdogs.

Env:
    HZ           colour rate cap, default 2.0
    DEPTH        1 (default) to publish depth, 0 to skip it
    DEPTH_HZ     depth rate cap, default 2.0
    COLOR_TOPIC  default /camera/camera/color/image_raw
    DEPTH_TOPIC  default /camera/camera/aligned_depth_to_color/image_raw
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# The depth camera_info topic name drive_replay hard-codes. Kept verbatim so a
# new bag drops into that pipeline; it is a name, not a claim about which rover.
REPLAY_DEPTH_INFO = '/rob_4/camera/depth/camera_info'


class BagFeeds(Node):
    def __init__(self):
        super().__init__('debug_color_throttle')
        self.period = 1.0 / float(os.environ.get('HZ', '2.0'))
        self.depth_period = 1.0 / float(os.environ.get('DEPTH_HZ', '2.0'))
        self.want_depth = os.environ.get('DEPTH', '1') == '1'
        color = os.environ.get('COLOR_TOPIC',
                               '/camera/camera/color/image_raw')
        depth = os.environ.get(
            'DEPTH_TOPIC',
            '/camera/camera/aligned_depth_to_color/image_raw')
        depth_info = depth.rsplit('/', 2)[0] + '/depth/camera_info'
        color_info = color.rsplit('/', 1)[0] + '/camera_info'

        self.last = {'color': 0.0, 'depth': 0.0, 'info': 0.0, 'cinfo': 0.0}
        self.n = {'color': 0, 'depth': 0}
        self.seen = {'color': 0, 'depth': 0}     # arrivals, before decimation
        self.t0 = time.monotonic()

        self.pub_color = self.create_publisher(
            CompressedImage, '/bag/color/compressed', 5)
        self.create_subscription(CompressedImage, color + '/compressed',
                                 self._on_color, qos_profile_sensor_data)

        self.pub_info = self.create_publisher(CameraInfo, REPLAY_DEPTH_INFO, 5)
        self.create_subscription(CameraInfo, depth_info, self._on_info,
                                 qos_profile_sensor_data)

        # The colour intrinsics. Cheap at 1 Hz and load-bearing: running the
        # ArUco detector offline against the bag is impossible without them,
        # and that is the difference between "we can retune marker_length
        # afterwards" and "the marker poses are whatever we guessed in the lab".
        self.pub_cinfo = self.create_publisher(
            CameraInfo, '/bag/color/camera_info', 5)
        self.create_subscription(CameraInfo, color_info, self._on_color_info,
                                 qos_profile_sensor_data)

        if self.want_depth:
            self.pub_depth = self.create_publisher(
                CompressedImage, '/bag/depth/compressed', 5)
            self.create_subscription(Image, depth, self._on_depth,
                                     qos_profile_sensor_data)

        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'{color}/compressed -> /bag/color/compressed @ '
            f'{1.0 / self.period:.1f} Hz'
            + (f'; {depth} -> /bag/depth/compressed @ '
               f'{1.0 / self.depth_period:.1f} Hz' if self.want_depth
               else '; depth OFF'))

    def _due(self, kind, period):
        t = time.monotonic()
        if t - self.last[kind] < period:
            return False
        self.last[kind] = t
        return True

    def _on_color(self, msg):
        self.seen['color'] += 1
        if self._due('color', self.period):
            self.pub_color.publish(msg)
            self.n['color'] += 1

    def _on_info(self, msg):
        # 30 Hz of camera_info cost more in the 2026-08-20 bag than /cmd_vel,
        # /tf and the IMU combined. Replay needs one per second at most.
        if self._due('info', 1.0):
            self.pub_info.publish(msg)

    def _on_color_info(self, msg):
        if self._due('cinfo', 1.0):
            self.pub_cinfo.publish(msg)

    def _on_depth(self, msg):
        self.seen['depth'] += 1
        if not self._due('depth', self.depth_period):
            return
        # Plain PNG bytes, NOT the compressedDepth transport format -- that one
        # prefixes a 12-byte header, and drive_replay decodes with a bare
        # cv2.imdecode(IMREAD_UNCHANGED), which would return garbage.
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warn(
                f'depth encoding {msg.encoding} is not 16UC1; skipping',
                once=True)
            return
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.step // 2)[:, :msg.width]
        ok, enc = cv2.imencode('.png', d, [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'png'
        out.data = enc.tobytes()
        self.pub_depth.publish(out)
        self.n['depth'] += 1

    def _report(self):
        if self.n['color'] == 0:
            # The compressed colour topic only exists if the image_transport
            # plugins are installed next to the RealSense driver. Silence here
            # means a bag with no camera, and nothing else says so.
            self.get_logger().warn(
                'no colour frames yet -- check '
                'ros2 topic list | grep color/image_raw/compressed')
            return
        el = max(time.monotonic() - self.t0, 1e-6)
        out_c, out_d = self.n['color'] / el, self.n['depth'] / el
        in_c, in_d = self.seen['color'] / el, self.seen['depth'] / el
        self.get_logger().info(
            f'colour {self.n["color"]} out @ {out_c:.1f} Hz (camera {in_c:.1f})'
            f'; depth {self.n["depth"]} out @ {out_d:.1f} Hz (camera {in_d:.1f})')
        # Decimation can only give camera_rate/N, so asking 10 Hz of a 15 fps
        # camera yields 7.5. Saying so beats discovering it in the bag.
        want_c = 1.0 / self.period
        if out_c < 0.8 * want_c:
            self.get_logger().warn(
                f'colour is {out_c:.1f} Hz, asked {want_c:.1f}. The camera is '
                f'supplying {in_c:.1f} fps and this only decimates: set the '
                f'RealSense colour profile to a whole multiple of {want_c:.0f}.')
        if self.want_depth:
            want_d = 1.0 / self.depth_period
            if out_d < 0.8 * want_d:
                self.get_logger().warn(
                    f'depth is {out_d:.1f} Hz, asked {want_d:.1f} '
                    f'(camera {in_d:.1f} fps).')


def main():
    rclpy.init()
    node = BagFeeds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
