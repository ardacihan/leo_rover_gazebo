#!/usr/bin/env python3
"""CompressedImage -> Image, header preserved. For offline replay only.

`aruco_detector` subscribes to `sensor_msgs/Image`, but the bag carries jpeg on
`/bag/color/compressed` — that is the whole point of the bag being small. This
sits between them during a replay.

The header must survive intact. The detector reads `header.frame_id` to know
which camera frame to look up against `map`, and `header.stamp` to look it up
at the right instant; a re-stamped or re-framed image silently places every
marker wrong instead of failing.

    python3 decompress_color.py --ros-args -p use_sim_time:=true

`image_transport republish compressed raw --ros-args -r in/compressed:=...`
does the same job if the plugins happen to be installed. This does not depend
on that.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class Decompressor(Node):
    def __init__(self):
        super().__init__('decompress_color')
        src = self.declare_parameter('in_topic', '/bag/color/compressed').value
        dst = self.declare_parameter('out_topic', '/bag/color/image_raw').value
        self.n = 0
        self.pub = self.create_publisher(Image, dst, 5)
        self.create_subscription(CompressedImage, src, self._cb,
                                 qos_profile_sensor_data)
        self.create_timer(20.0, lambda: self.get_logger().info(
            f'{self.n} frames -> {dst}'))
        self.get_logger().info(f'{src} -> {dst}')

    def _cb(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                           cv2.IMREAD_COLOR)
        if bgr is None:
            return
        out = Image()
        out.header = msg.header          # frame_id AND stamp, untouched
        out.height, out.width = bgr.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.pub.publish(out)
        self.n += 1


def main():
    rclpy.init()
    node = Decompressor()
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
