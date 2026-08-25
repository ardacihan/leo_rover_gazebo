#!/usr/bin/env python3
"""Presentation-grade camera sampling at negligible cost.

Subscribes the RealSense's *already-encoded* JPEG stream (compressed
image_transport topic) and writes every Nth frame straight to disk --
no decode, no re-encode, one small best-effort subscription. At 2 Hz and
~100 KB/frame a 30-minute run is ~350 MB and ~0 CPU.

Env overrides:
  CAM_TOPIC   (default /rob_4/camera/color/image_raw/compressed)
  OUT_DIR     (default ~/leo_nav2_ws/runs/current_media)
  SAMPLE_HZ   (default 2.0)
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CamSampler(Node):
    def __init__(self):
        super().__init__("presentation_cam_sampler")
        self.out = os.path.expanduser(
            os.environ.get("OUT_DIR", "~/leo_nav2_ws/runs/current_media")
        )
        os.makedirs(self.out, exist_ok=True)
        self.period = 1.0 / float(os.environ.get("SAMPLE_HZ", "2.0"))
        self.last = 0.0
        self.n = 0
        topic = os.environ.get(
            "CAM_TOPIC", "/rob_4/camera/color/image_raw/compressed"
        )
        self.create_subscription(
            CompressedImage, topic, self.cb, qos_profile_sensor_data
        )
        self.get_logger().info(f"sampling {topic} -> {self.out}")

    def cb(self, msg):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        self.last = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        path = os.path.join(self.out, "cam_%06d_%.3f.jpg" % (self.n, stamp))
        with open(path, "wb") as f:
            f.write(bytes(msg.data))
        self.n += 1


def main():
    rclpy.init()
    node = CamSampler()
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
