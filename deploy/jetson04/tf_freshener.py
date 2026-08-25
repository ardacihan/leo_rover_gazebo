#!/usr/bin/env python3
"""Re-stamp map->odom AND odom->base_footprint with current Jetson time.

Humble's slam_toolbox stamps map->odom with the last *processed* scan time and
only processes scans after >=minimum_travel motion, so a stationary robot's
transform stamp freezes. Consumers that resolve "latest common time"
(m-explore's costmap client, getRobotPose) then fail with
extrapolation-into-the-past once the odom->base history window passes.
Newer slam_toolbox fixes this with restamp_tf; this node is that fix for
Humble: echo the newest map->odom value, stamped now, at 20 Hz.

odom->base_footprint has the mirror problem: its stamps come from the rover
SBC's clock via the firmware odometry, which drifts behind the Jetson clock
over a run. Lookups at Jetson-now (explore's getRobotPose, the aruco
detector at image stamps) then fail with extrapolation-into-the-FUTURE --
this killed frontier goals ~35 min into the 2026-08-25 run and blocks
marker registration. Same cure: echo the newest value stamped now.

If the firmware genuinely dies the echoed odom freezes at its last value;
the stack's other watchdogs (velocity guard on /wheel_odom, collision
monitor on /scan_filtered) are the freshness authorities, not TF stamps.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class TfFreshener(Node):
    def __init__(self):
        super().__init__("map_odom_tf_freshener")
        self.latest_map_odom = None
        self.latest_odom_base = None
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, "/tf", self.cb, 50)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "re-stamping map->odom and odom->base_footprint at 20 Hz"
        )

    def cb(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                # Skip our own re-broadcasts (same value, newer stamp); slam's
                # copies are the source of truth.
                if self.latest_map_odom is not None and (
                    tf.transform == self.latest_map_odom.transform
                    and tf.header.stamp.sec >= self.latest_map_odom.header.stamp.sec
                ):
                    continue
                self.latest_map_odom = tf
            elif tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                # An identical transform is either our own echo or a
                # stationary robot repeating itself; either way the held
                # value is already right, and skipping breaks the feedback
                # loop with our own broadcasts.
                if self.latest_odom_base is not None and (
                    tf.transform == self.latest_odom_base.transform
                ):
                    continue
                self.latest_odom_base = tf

    def tick(self):
        now = self.get_clock().now().to_msg()
        out = []
        for latest in (self.latest_map_odom, self.latest_odom_base):
            if latest is None:
                continue
            fresh = TransformStamped()
            fresh.header.stamp = now
            fresh.header.frame_id = latest.header.frame_id
            fresh.child_frame_id = latest.child_frame_id
            fresh.transform = latest.transform
            out.append(fresh)
        if out:
            self.broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = TfFreshener()
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
