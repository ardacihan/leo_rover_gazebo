"""Depth-cloud filter feeding the costmap's camera obstacle layer.

The costmap must never show an obstacle that is not there: a phantom blob
flickering on open floor stalls the planner just as surely as a wall. This
node makes the camera trustworthy:

    * transform to a level world frame and cut the floor plane and
      everything above the mast, using the measured z, not a guess;
    * decimate to a 3 cm voxel grid (the costmap is 2.5 cm -- finer detail
      is wasted CPU);
    * drop voxels with a sparse neighbourhood: isolated specks are depth
      noise, real structure (chair legs, table pedestals, crossbars)
      occupies adjacent voxels in quantity;
    * temporal voting in the odom frame: a voxel must be seen in
      `persistence_min_hits` of the last `persistence_frames` clouds before
      it may mark. A single-frame return can never become an obstacle.

Points are republished **in the sensor's own frame** so the ObstacleLayer
still raytraces from the true camera origin.

    ros2 run leo_nav2_exploration cloud_filter

The defaults match config/real/nav2.yaml's camera_obstacle_layer; the replay
harness runs the same node against the bag-derived cloud.
"""
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def xyz_of(msg):
    """View the cloud's x,y,z as an (N,3) float32 array without copying."""
    offsets = {f.name: f.offset for f in msg.fields}
    if any(n not in offsets for n in 'xyz'):
        return None
    n = msg.width * msg.height
    step = msg.point_step
    # Fast path: word-aligned consecutive x,y,z (any point_step, e.g. the
    # Jetson NEON filter pads points to 16 bytes) — view rows as float32
    # words and slice, which stays a copy-free view.
    if (step % 4 == 0 and offsets['x'] % 4 == 0
            and offsets['y'] == offsets['x'] + 4
            and offsets['z'] == offsets['x'] + 8):
        words = np.frombuffer(msg.data, dtype=np.uint8)[:n * step].view(
            np.float32).reshape(n, step // 4)
        i = offsets['x'] // 4
        return words[:, i:i + 3]
    data = np.frombuffer(msg.data, dtype=np.uint8)
    data = data[:n * step].reshape(n, step)
    return np.stack([
        np.ascontiguousarray(data[:, offsets[c]:offsets[c] + 4])
        .view(np.float32).ravel()
        for c in 'xyz'], axis=1)


class CloudFilter(Node):

    def __init__(self):
        super().__init__('cloud_filter')
        p = self.declare_parameters('', [
            ('input_topic', '/camera/camera/depth/color/points'),
            ('output_topic', '/camera_points_filtered'),
            # The voting grid must not move with the robot, so filtering
            # happens in odom; z is still floor-referenced there.
            ('target_frame', 'odom'),
            ('min_z', 0.08),
            ('max_z', 0.65),
            ('max_range', 2.5),
            ('voxel', 0.03),
            ('min_neighbor_points', 8),
            ('persistence_frames', 3),
            ('persistence_min_hits', 2),
            ('max_rate', 6.0),
            ('stride', 2),
        ])
        g = lambda n: self.get_parameter(n).value
        self.min_z, self.max_z = float(g('min_z')), float(g('max_z'))
        self.max_range = float(g('max_range'))
        self.voxel = float(g('voxel'))
        self.min_neighbors = int(g('min_neighbor_points'))
        self.persist_n = max(1, int(g('persistence_frames')))
        self.persist_hits = min(max(1, int(g('persistence_min_hits'))),
                                self.persist_n)
        self.history = deque(maxlen=self.persist_n - 1)
        self.stride = max(1, int(g('stride')))
        self.min_period = 1.0 / float(g('max_rate'))
        self.target = str(g('target_frame'))
        self.last_stamp = None

        self.tf_buffer = Buffer()
        # Dedicated spin thread: on this rover the 30 Hz cloud callbacks
        # starve a shared executor and the TF buffer falls seconds behind,
        # failing every lookup (observed 2026-08-21).
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, str(g('output_topic')), 2)
        # Raw + depth-1 subscription: deserializing every 30 Hz cloud just to
        # drop it in the stamp rate-limit holds the GIL long enough to starve
        # the TF thread (observed 2026-08-21). Keep only the newest sample in
        # DDS and deserialize at most at the processed rate.
        self._last_rx = None
        raw_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, str(g('input_topic')),
                                 self.on_cloud_raw, raw_qos, raw=True)
        self.get_logger().info(
            f"cloud_filter {g('input_topic')} -> {g('output_topic')} "
            f"z=[{self.min_z},{self.max_z}]m in {self.target}, "
            f"voxel {self.voxel} m, min neighbourhood {self.min_neighbors}, "
            f"voting {self.persist_hits}/{self.persist_n} frames")

    def on_cloud_raw(self, data):
        now = self.get_clock().now()
        if (self._last_rx is not None
                and (now - self._last_rx).nanoseconds < self.min_period * 1e9):
            return
        self._last_rx = now
        self.on_cloud(deserialize_message(bytes(data), PointCloud2))

    def on_cloud(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp)
        if (self.last_stamp is not None
                and (t - self.last_stamp).nanoseconds < self.min_period * 1e9):
            return
        pts = xyz_of(msg)
        if pts is None:
            return
        pts = pts[::self.stride]
        pts = pts[np.isfinite(pts).all(axis=1)]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target, msg.header.frame_id, t,
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warn(f'no TF: {exc}', throttle_duration_sec=5.0)
            return
        self.last_stamp = t
        tr = tf.transform.translation
        R = quat_mat(tf.transform.rotation)
        world = pts @ R.T + np.array([tr.x, tr.y, tr.z])

        rng = np.hypot(world[:, 0] - tr.x, world[:, 1] - tr.y)
        keep = ((world[:, 2] > self.min_z) & (world[:, 2] < self.max_z)
                & (rng < self.max_range))
        pts, world = pts[keep], world[keep]
        if len(pts):
            vox = np.floor(world / self.voxel).astype(np.int64)
            key = (vox[:, 0] * 73856093) ^ (vox[:, 1] * 19349663) ^ (vox[:, 2] * 83492791)
            order = np.argsort(key)
            key_s, vox_s = key[order], vox[order]
            uniq_pos = np.nonzero(np.r_[True, key_s[1:] != key_s[:-1]])[0]
            counts = np.diff(np.r_[uniq_pos, len(key_s)])
            occupied = {}
            for idx, c in zip(uniq_pos, counts):
                occupied[tuple(vox_s[idx])] = int(c)
            first_index = order[uniq_pos]

            frame_set = set(occupied)
            need_prior = self.persist_hits - 1  # this frame is one hit
            keep_idx = []
            for idx, c in zip(uniq_pos, counts):
                v = vox_s[idx]
                key3 = (int(v[0]), int(v[1]), int(v[2]))
                if need_prior > 0:
                    prior = sum(1 for s in self.history if key3 in s)
                    if prior < need_prior:
                        keep_idx.append(False)
                        continue
                total = 0
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            total += occupied.get((v[0] + dx, v[1] + dy,
                                                   v[2] + dz), 0)
                            if total >= self.min_neighbors:
                                break
                        if total >= self.min_neighbors:
                            break
                    if total >= self.min_neighbors:
                        break
                keep_idx.append(total >= self.min_neighbors)
            pts = pts[first_index[np.array(keep_idx, bool)]]
            self.history.append(frame_set)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(pts)
        out.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyz')]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * len(pts)
        out.is_dense = True
        out.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
