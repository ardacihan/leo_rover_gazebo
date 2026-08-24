#!/usr/bin/env python3
"""Publish the *estimated* leo1/map -> leo2/map once alignment is trusted.

Without this node the two rovers share a merged map but not a transform tree:
`shared_map_merger` composites the grids internally and the aligners broadcast
only diagnostic frames (`leo2/map_estimated`, `leo2/map_grid_estimated`), so
nothing connects `leo1/map` to `leo2/map`. Everything that coordinates the
rovers is a TF lookup and therefore silently fails:

* `frontier_explorer._peer_xy`  -> `{our map} -> {peer}/base_link`
* `frontier_explorer._get_common_offset` -> `map -> leo{i}/map`

With no such transform, `_select_frontier` finds no peers, falls through to the
locally best frontier, and "coordinated" mode degrades to independent without
saying so. The old fix was `map_merge_leo.launch.py`, which publishes *identity*
`map -> leo{i}/map` statics -- correct only under ground-truth odometry, which
is exactly the cheat this run is meant to remove.

So: consume the accepted transform the merger already trusts, gate it on the
same confidence, and broadcast it as real TF. `leo2/map` gains a parent
(`leo1/map`); it keeps its own child `leo2/odom` from leo2's slam_toolbox, so
the tree stays single-parent and rooted at `leo1/map`.

Two properties that matter:

* **Nothing is published before alignment.** A wrong transform is worse than
  no transform -- rovers would coordinate over a bogus shared frame and the
  merged map would be confidently wrong. Below `min_confidence` this node is
  silent and both rovers explore independently, which is the correct
  pre-rendezvous behaviour.
* **The estimate keeps improving.** It republishes the latest accepted
  transform at `publish_rate_hz`, so a converging alignment reaches the
  explorers instead of being frozen at its first and worst value.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster


class AlignmentTfBridge(Node):
    def __init__(self):
        super().__init__("alignment_tf_bridge")

        # The merger's accepted (tag + map-matching fused) estimate. Falling
        # back to the tag-only topic keeps this useful when map_based_aligner
        # is not running.
        self.declare_parameter("transform_topic", "/map_based_transform/leo2_to_leo1")
        self.declare_parameter("confidence_topic", "/alignment_confidence")
        self.declare_parameter("fallback_transform_topic",
                               "/estimated_transform/leo2_to_leo1")
        self.declare_parameter("fallback_confidence_topic",
                               "/tag_alignment_confidence")
        self.declare_parameter("use_fallback", True)
        self.declare_parameter("parent_frame", "leo1/map")
        self.declare_parameter("child_frame", "leo2/map")
        self.declare_parameter("min_confidence", 0.5)
        # Once locked, do not go silent on a transient confidence dip: the
        # rovers would lose each other mid-plan and thrash between coordinated
        # and independent allocation. Drop the lock only if confidence stays
        # below `unlock_confidence`.
        self.declare_parameter("unlock_confidence", 0.2)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("status_topic", "/alignment_locked")
        # Republish the transform this node has actually vetted, so the map
        # merger can consume the SAME decision instead of re-deriving its own.
        # Without this the veto protects only the TF tree: shared_map_merger
        # subscribes to /map_based_transform directly with its own confidence
        # gate, and on depot_world 2026-08-24 it fused leo2 under a
        # 180-degree-flipped grid match and produced a doubled world, while
        # this node was correctly refusing to publish that very transform.
        self.declare_parameter("vetted_transform_topic",
                               "/vetted_transform/leo2_to_leo1")
        self.declare_parameter("vetted_confidence_topic",
                               "/vetted_alignment_confidence")
        self.declare_parameter("status_log_period_sec", 10.0)
        # Cross-check: when the tag estimate and the grid-matching estimate
        # both exist and disagree badly, neither is trustworthy and NOTHING is
        # published. Confidence cannot catch this on its own -- it is built on
        # residuals, which measure self-consistency rather than correctness.
        # Measured on 2026-08-23: three common landmarks fitted each other to
        # 0.06-0.09 m residuals, reported 0.82 confidence, and were 114 degrees
        # wrong, because leo2's SLAM had lost its heading. The tag and map
        # estimates disagreed by ~46 degrees at the time; that disagreement was
        # the only signal available that something was wrong.
        # Grid matching alone must never lock the frame. The mission is that
        # the rovers discover each other by both seeing the same wall markers;
        # map matching is the cross-check, not the primary. An office or depot
        # is full of rectilinear self-similarity, so a grid matcher will
        # cheerfully find a confident 180-degree-flipped match: on depot_world
        # 2026-08-24 it locked at 0.49 confidence with ZERO common tags and a
        # yaw error of 179.9 degrees. With no tag estimate there is also no
        # second opinion, so `require_agreement` cannot save it either.
        self.declare_parameter("require_tag_evidence", True)
        self.declare_parameter("require_agreement", True)
        self.declare_parameter("max_disagreement_xy", 2.0)
        self.declare_parameter("max_disagreement_yaw_deg", 25.0)

        self.parent = str(self.get_parameter("parent_frame").value)
        self.child = str(self.get_parameter("child_frame").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.unlock_conf = float(self.get_parameter("unlock_confidence").value)
        self.require_tag_evidence = bool(
            self.get_parameter("require_tag_evidence").value)
        self.require_agreement = bool(self.get_parameter("require_agreement").value)
        self.max_dis_xy = float(self.get_parameter("max_disagreement_xy").value)
        self.max_dis_yaw = math.radians(
            float(self.get_parameter("max_disagreement_yaw_deg").value))
        self._disagree = None

        self._primary: Optional[Tuple[float, float, float]] = None
        self._fallback: Optional[Tuple[float, float, float]] = None
        self._primary_conf = 0.0
        self._fallback_conf = 0.0
        self._locked = False
        self._lock_time: Optional[float] = None
        self._last_log = 0.0

        self.create_subscription(
            TransformStamped, str(self.get_parameter("transform_topic").value),
            lambda m: self._on_transform(m, primary=True), 10)
        self.create_subscription(
            Float32, str(self.get_parameter("confidence_topic").value),
            lambda m: self._on_confidence(m, primary=True), 10)
        if bool(self.get_parameter("use_fallback").value):
            self.create_subscription(
                TransformStamped,
                str(self.get_parameter("fallback_transform_topic").value),
                lambda m: self._on_transform(m, primary=False), 10)
            self.create_subscription(
                Float32,
                str(self.get_parameter("fallback_confidence_topic").value),
                lambda m: self._on_confidence(m, primary=False), 10)

        self.broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(
            Bool, str(self.get_parameter("status_topic").value), 10)
        self.vetted_pub = self.create_publisher(
            TransformStamped,
            str(self.get_parameter("vetted_transform_topic").value), 10)
        self.vetted_conf_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("vetted_confidence_topic").value), 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f"alignment_tf_bridge: {self.parent} -> {self.child} once "
            f"confidence >= {self.min_conf} (unlock below {self.unlock_conf})")

    # ------------------------------------------------------------- callbacks

    @staticmethod
    def _to_xy_yaw(msg: TransformStamped) -> Tuple[float, float, float]:
        t = msg.transform
        return (float(t.translation.x), float(t.translation.y),
                2.0 * math.atan2(t.rotation.z, t.rotation.w))

    def _on_transform(self, msg: TransformStamped, primary: bool):
        if primary:
            self._primary = self._to_xy_yaw(msg)
        else:
            self._fallback = self._to_xy_yaw(msg)

    def _on_confidence(self, msg: Float32, primary: bool):
        if primary:
            self._primary_conf = float(msg.data)
        else:
            self._fallback_conf = float(msg.data)

    # ------------------------------------------------------------------ tick

    def _best(self):
        """(estimate, confidence, source) -- the accepted estimate wins."""
        if self._primary is not None:
            return self._primary, self._primary_conf, "map_based"
        if self._fallback is not None:
            return self._fallback, self._fallback_conf, "tag_only"
        return None, 0.0, "none"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _disagreement(self):
        """(d_xy, d_yaw) between the tag and grid estimates, or None."""
        if self._primary is None or self._fallback is None:
            return None
        px, py, pyaw = self._primary
        fx, fy, fyaw = self._fallback
        dyaw = abs(math.atan2(math.sin(pyaw - fyaw), math.cos(pyaw - fyaw)))
        return math.hypot(px - fx, py - fy), dyaw

    def _tick(self):
        estimate, conf, source = self._best()

        # No tag evidence -> no lock, however confident the grid match looks.
        if self.require_tag_evidence and self._fallback is None:
            if self._locked:
                self._locked = False
                self.get_logger().warn(
                    "lost tag evidence; dropping the lock rather than trusting "
                    "grid matching alone")
            self._publish_status(False)
            self._log_throttled(
                "withholding transform: no tag estimate yet "
                f"(grid-only confidence {self._primary_conf:.2f}); the rovers "
                "have not seen common markers")
            return

        # Two independent estimates that disagree are evidence against both.
        self._disagree = self._disagreement()
        if self.require_agreement and self._disagree is not None:
            d_xy, d_yaw = self._disagree
            if d_xy > self.max_dis_xy or d_yaw > self.max_dis_yaw:
                # They disagree, so the fused estimate is not trustworthy. But
                # tags are the primary sensor here -- the rovers are meant to
                # find each other by seeing the same markers, and grid matching
                # is only the cross-check. A rectilinear world gives the grid
                # matcher confident 180-degree flips (depot_world 2026-08-24:
                # grid 9.58 m / 179.9 deg wrong while the tags were 0.75 m /
                # 0.3 deg right). So prefer the tag estimate on its own when it
                # is confident enough, and withhold entirely only when it is
                # not -- withholding a good tag estimate costs all coordination.
                if self._fallback_conf >= self.min_conf:
                    estimate, conf, source = (
                        self._fallback, self._fallback_conf, "tag_only(disagreement)")
                    self._log_throttled(
                        f"tag vs grid disagree by {d_xy:.2f} m / "
                        f"{math.degrees(d_yaw):.1f} deg -- trusting the TAG "
                        f"estimate alone (confidence {self._fallback_conf:.2f})")
                else:
                    if self._locked:
                        self._locked = False
                        self.get_logger().warn(
                            "tag and grid alignment disagree "
                            f"({d_xy:.2f} m, {math.degrees(d_yaw):.1f} deg) and "
                            "the tag estimate is weak -- dropping the lock; a "
                            "confident-but-wrong transform is worse than none")
                    self._publish_status(False)
                    self._log_throttled(
                        f"withholding transform: tag vs grid disagree by "
                        f"{d_xy:.2f} m / {math.degrees(d_yaw):.1f} deg "
                        f"(limits {self.max_dis_xy} m / "
                        f"{math.degrees(self.max_dis_yaw):.0f} deg) and tag "
                        f"confidence is only {self._fallback_conf:.2f}")
                    return

        if estimate is None:
            self._publish_status(False)
            self._log_throttled("waiting for an alignment estimate")
            return

        if self._locked:
            if conf < self.unlock_conf:
                self._locked = False
                self.get_logger().warn(
                    f"alignment lost (confidence {conf:.2f} < "
                    f"{self.unlock_conf}); stopping {self.parent} -> "
                    f"{self.child} and reverting to independent exploration")
        elif conf >= self.min_conf:
            self._locked = True
            self._lock_time = self._now()
            x, y, yaw = estimate
            self.get_logger().info(
                f"alignment LOCKED at t={self._lock_time:.1f}s via {source}: "
                f"{self.child} is ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg) "
                f"in {self.parent}, confidence {conf:.2f}")

        self._publish_status(self._locked)
        if not self._locked:
            self._log_throttled(
                f"unaligned: best confidence {conf:.2f} from {source} "
                f"(need {self.min_conf})")
            return

        x, y, yaw = estimate
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent
        tf.child_frame_id = self.child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(tf)
        # Same message to the merger, so the grid it fuses and the frame the
        # explorers coordinate in can never diverge.
        self.vetted_pub.publish(tf)
        conf_msg = Float32()
        conf_msg.data = float(conf)
        self.vetted_conf_pub.publish(conf_msg)
        self._log_throttled(
            f"aligned via {source}: ({x:.2f}, {y:.2f}, "
            f"{math.degrees(yaw):.1f} deg) confidence {conf:.2f}")

    def _publish_status(self, locked: bool):
        msg = Bool()
        msg.data = bool(locked)
        self.status_pub.publish(msg)

    def _log_throttled(self, text: str):
        period = float(self.get_parameter("status_log_period_sec").value)
        now = self._now()
        if now - self._last_log >= period:
            self._last_log = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentTfBridge()
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
