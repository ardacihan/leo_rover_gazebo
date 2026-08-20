#!/usr/bin/env python3
"""Record the local costmap the recovery behaviours actually read.

`behavior_server` checks `local_costmap/costmap_raw` with the robot's footprint
before it will move, and logs `Collision Ahead` when that check fails. The log
line says nothing about *what* it saw, so this writes the costmap out as a
picture once a second, with the footprint drawn on it and the cost legend the
checker uses:

    254 LETHAL                 an obstacle
    253 INSCRIBED_INFLATED     within the robot's inscribed radius of one --
                               the checker treats this as a collision, so any
                               footprint cell here means "cannot move"
    128-252 inflation gradient the planner's discouragement, not a collision
    0   free                   0 in the raw costmap
    255 NO_INFORMATION         unknown

Frames are 160x160 for a 4 m window at 2.5 cm, so a whole run is a few MB.

    python3 costmap_recorder.py <outdir> [period_s] [--ros-args -p robot:=leo1]

`index.csv` carries the pose and a moving/stalled flag per frame, so the frames
around a stall can be found without watching the whole run.
"""

import math
import os
import sys

import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import tf2_ros

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

LETHAL, INSCRIBED, NO_INFO = 254, 253, 255


def colourise(grid):
    """Cost grid -> BGR, with the checker's thresholds visually distinct."""
    h, w = grid.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (60, 60, 60)                                  # free: dark grey
    unknown = grid == NO_INFO
    infl = (grid >= 1) & (grid < INSCRIBED)
    # Inflation gradient: dim blue rising with cost. Discouraging, not blocking.
    scaled = np.clip(grid.astype(np.int32), 0, 252)
    img[infl] = np.stack([80 + scaled[infl] * 0.6, 40 + scaled[infl] * 0.2,
                          np.zeros(infl.sum())], axis=-1).astype(np.uint8)
    img[grid == INSCRIBED] = (0, 140, 255)                 # orange: blocks recovery
    img[grid == LETHAL] = (0, 0, 255)                      # red: obstacle
    img[unknown] = (110, 110, 110)                         # mid grey
    return img


class CostmapRecorder(Node):

    def __init__(self, outdir, period):
        super().__init__('costmap_recorder')
        self.declare_parameter('robot', 'leo1')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('scale', 3)
        self.declare_parameter('stall_speed', 0.02)
        self.declare_parameter('footprint_half', 0.21)

        g = lambda n: self.get_parameter(n).value
        self.robot = g('robot')
        self.scale = int(g('scale'))
        self.stall_speed = float(g('stall_speed'))
        self.half = float(g('footprint_half'))

        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.index = open(os.path.join(outdir, 'index.csv'), 'w')
        self.index.write('frame,t,x,y,yaw,speed,stalled,'
                         'footprint_max_cost,footprint_blocked\n')

        self.costmap = None
        self.scan = None
        self.speed = 0.0
        self.n = 0
        self.miscentred = 0

        # The pose MUST come from the same TF tree the costmap is built in.
        # Reading it from /leo1/odom instead looks right and is wrong: that
        # topic is world-anchored, while the costmap is anchored on the EKF's
        # odom->base_link, and the two diverge by the odometry drift -- metres,
        # late in a run. The footprint then gets sampled somewhere the robot is
        # not. With rolling_window the robot is always at the centre of the
        # window, which makes that mistake self-detecting; see `miscentred`.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, g('costmap_topic'), self.on_costmap, qos)
        self.create_subscription(Odometry, f'/{self.robot}/odom', self.on_odom, 20)
        self.create_subscription(LaserScan, f'/{self.robot}/scan_filtered',
                                 self.on_scan, 10)
        self.create_timer(period, self.snap)
        self.get_logger().info(
            f"costmap_recorder -> {outdir} every {period}s from {g('costmap_topic')}")

    def on_costmap(self, msg):
        self.costmap = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        # Only the speed is taken from odometry; the pose comes from TF.
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def costmap_pose(self):
        """Robot pose in the costmap's own frame, or None."""
        if self.costmap is None:
            return None
        frame = self.costmap.header.frame_id or f'{self.robot}/odom'
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, f'{self.robot}/base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'no TF {frame} <- base_link: {exc}',
                                   throttle_duration_sec=10.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)


    @staticmethod
    def _sample(grid, w, h, cells):
        best = -1
        for px, py in cells:
            if 0 <= px < w and 0 <= py < h:
                c = int(grid[py, px])
                if c != NO_INFO and c > best:
                    best = c
        return best

    def polygon_cost(self, grid, cx, cy, yaw, res, w, h):
        """Max cost on the rotated footprint outline, as Nav2's checker does."""
        pts = []
        for dx, dy in ((self.half, self.half), (self.half, -self.half),
                       (-self.half, -self.half), (-self.half, self.half)):
            rx = dx * math.cos(yaw) - dy * math.sin(yaw)
            ry = dx * math.sin(yaw) + dy * math.cos(yaw)
            pts.append((cx + rx / res, cy + ry / res))
        cells = []
        for i in range(4):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % 4]
            n = max(2, int(math.hypot(x1 - x0, y1 - y0)) + 1)
            for k in range(n + 1):
                cells.append((int(round(x0 + (x1 - x0) * k / n)),
                              int(round(y0 + (y1 - y0) * k / n))))
        return self._sample(grid, w, h, cells)

    def disc_cost(self, grid, cx, cy, radius_m, res, w, h):
        """Max cost anywhere within radius -- the area an in-place spin sweeps."""
        r = int(math.ceil(radius_m / res))
        cells = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    cells.append((cx + dx, cy + dy))
        return self._sample(grid, w, h, cells)

    # ------------------------------------------------------------------ draw

    def snap(self):
        if self.costmap is None or cv2 is None:
            return
        pose = self.costmap_pose()
        if pose is None:
            return
        self.pose = pose
        m = self.costmap.metadata
        w, h, res = m.size_x, m.size_y, m.resolution
        grid = np.array(self.costmap.data, dtype=np.uint8).reshape(h, w)

        # Costmap origin is its corner in the costmap's own (odom) frame.
        ox, oy = m.origin.position.x, m.origin.position.y
        x, y, yaw = self.pose

        def to_px(wx, wy):
            return int((wx - ox) / res), int((wy - oy) / res)

        cx, cy = to_px(x, y)

        # Two different questions, and they have different answers:
        #
        #   max_cost  cost under the footprint at the CURRENT heading -- what
        #             stops a BackUp (a pure translation).
        #   rot_cost  cost anywhere within the circumscribed radius -- what an
        #             in-place Spin sweeps. A 0.42 m square has an inscribed
        #             radius of 0.21 m but a circumscribed radius of 0.297 m,
        #             so rotating reaches 9 cm further out than standing still.
        #             The inflation layer paints INSCRIBED (253) within 0.21 m
        #             of an obstacle, so a robot with 0.25 m of clearance is
        #             free to drive and forbidden to turn.
        max_cost = self.polygon_cost(grid, cx, cy, yaw, res, w, h)
        rot_cost = self.disc_cost(grid, cx, cy, self.half * math.sqrt(2.0),
                                  res, w, h)
        blocked = int(max_cost >= INSCRIBED)
        rot_blocked = int(rot_cost >= INSCRIBED)

        # Self-check: a rolling window is always centred on the robot. If the
        # footprint is not near the middle, the pose and the costmap are in
        # different frames and every cost sampled below is meaningless.
        if abs(cx - w / 2) > 4 or abs(cy - h / 2) > 4:
            self.miscentred += 1
            self.get_logger().warn(
                f'robot at cell ({cx},{cy}) but window centre is '
                f'({w // 2},{h // 2}) -- pose and costmap frames disagree',
                throttle_duration_sec=10.0)

        img = colourise(grid)
        img = cv2.resize(img, (w * self.scale, h * self.scale),
                         interpolation=cv2.INTER_NEAREST)
        s = self.scale

        if self.scan is not None:
            for i, rng in enumerate(self.scan.ranges):
                if not math.isfinite(rng) or rng <= self.scan.range_min:
                    continue
                a = self.scan.angle_min + i * self.scan.angle_increment + yaw
                px, py = to_px(x + rng * math.cos(a), y + rng * math.sin(a))
                if 0 <= px < w and 0 <= py < h:
                    cv2.circle(img, (px * s, py * s), 1, (0, 255, 255), -1)
        # Circumscribed circle: the area an in-place rotation sweeps.
        cv2.circle(img, (cx * s, cy * s),
                   int(self.half * math.sqrt(2.0) / res * s), (200, 200, 200), 1)

        corners = []
        for dx, dy in ((self.half, self.half), (self.half, -self.half),
                       (-self.half, -self.half), (-self.half, self.half)):
            wx = x + dx * math.cos(yaw) - dy * math.sin(yaw)
            wy = y + dx * math.sin(yaw) + dy * math.cos(yaw)
            px, py = to_px(wx, wy)
            corners.append([px * s, py * s])
        # green: free to drive and turn. orange: can drive, cannot spin.
        # red: the checker refuses any motion.
        colour = ((0, 0, 255) if blocked
                  else (0, 165, 255) if rot_blocked else (0, 255, 0))
        cv2.polylines(img, [np.array(corners, np.int32)], True, colour, 2)
        hx, hy = to_px(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw))
        cv2.line(img, (cx * s, cy * s), (hx * s, hy * s), colour, 2)

        stalled = int(self.speed < self.stall_speed)
        t = self.get_clock().now().nanoseconds * 1e-9
        # Costmap row 0 is -y, so flip before labelling -- otherwise the text
        # comes out mirrored along with the map.
        img = cv2.flip(img, 0)
        label = (f'f{self.n:05d} t={t:.0f} v={self.speed:.3f} '
                 f'fp={max_cost}{"!" if blocked else ""} '
                 f'rot={rot_cost}{" NOSPIN" if rot_blocked and not blocked else ""}')
        cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
        cv2.putText(img, label, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imwrite(os.path.join(self.outdir, f'f{self.n:05d}.png'), img)
        self.index.write(f'{self.n},{t:.2f},{x:.3f},{y:.3f},{yaw:.3f},'
                         f'{self.speed:.3f},{stalled},{max_cost},{blocked}\n')
        self.index.flush()
        self.n += 1


def main(args=None):
    argv = sys.argv[1:]
    positional = [a for a in argv if not a.startswith('-')]
    outdir = positional[0] if positional else 'costmaps'
    period = float(positional[1]) if len(positional) > 1 else 1.0

    rclpy.init(args=args)
    node = CostmapRecorder(outdir, period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.index.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
