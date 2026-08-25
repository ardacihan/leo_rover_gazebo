"""Frontier-based autonomous exploration + item search for the Leo Rover.

Pipeline:
1. EXPLORING  - frontier goals until the lidar map is complete (PR1-hardened:
   snapped + pre-validated goals, TTL blacklist, hysteresis, stall ladder,
   watchdog).
2. SWEEPING   - viewpoint goals until the *camera* has observed the walls
   (item search needs camera coverage, not just lidar coverage).
3. VERIFY     - on a first item detection, drive to a standoff viewpoint and
   re-observe to confirm; confirmed items go to the registry.
4. RETURNING/SAVING - back to start, save map, report found items.

Multi-robot ready: namespace-parameterized, frontier claims published on
/exploration_claims, found items on ~/found_items (JSON).
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Bool, String as StringMsg
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .camera_coverage import CameraCoverageTracker, yaw_from_quat
from .coordination import coordinated_allocation
from .observations import distinct_view
from .selection import sweep_candidate_metrics

FREE_MAX = 50      # occupancy values below this count as free
UNKNOWN = -1

STATE_EXPLORING = 'exploring'
STATE_SWEEPING = 'sweeping'
STATE_RETURNING = 'returning'
STATE_SAVING = 'saving'
STATE_DONE = 'done'

STATUS_SUCCEEDED = 4

GOAL_FRONTIER = 'frontier'
GOAL_SWEEP = 'sweep'
GOAL_VERIFY = 'verify'


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_name', 'leo1')
        self.declare_parameter('robot_base_frame', 'leo1/base_link')
        self.declare_parameter('camera_frame', 'leo1/sensor_camera_link')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('planner_frequency', 0.25)
        self.declare_parameter('min_frontier_size', 0.5)    # meters
        self.declare_parameter('potential_scale', 3.0)      # distance weight
        self.declare_parameter('gain_scale', 1.0)           # size weight
        self.declare_parameter('progress_timeout', 30.0)    # seconds
        self.declare_parameter('blacklist_radius', 0.6)     # meters
        # A banned frontier regrows. Ban a *point* with a 0.6 m radius and the
        # next planning cycle finds an almost identical frontier just outside
        # it, along the same unreachable wall -- on depot 2026-08-24, 11
        # distinct frontiers produced 21 failed goals this way. Each repeat
        # strike widens the banned region, so a wall that keeps failing is
        # eventually excluded as an area rather than as a dot.
        self.declare_parameter('blacklist_growth', 0.9)     # metres per strike
        self.declare_parameter('blacklist_max_radius', 3.0)
        self.declare_parameter('blacklist_ttl', 90.0)       # seconds
        self.declare_parameter('blacklist_long_ttl', 900.0)
        self.declare_parameter('blacklist_max_strikes', 3)
        self.declare_parameter('replan_distance', 0.7)      # meters
        self.declare_parameter('goal_commit_time', 10.0)    # seconds
        self.declare_parameter('switch_margin', 1.25)       # score ratio
        self.declare_parameter('validate_goals', True)
        self.declare_parameter('planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('skip_ttl', 20.0)            # no-path skip
        self.declare_parameter('claim_radius', 1.5)         # meters
        # Frontiers outside the building are the single biggest waste in a run.
        # A thin outer wall leaks a few lidar rays, leaving unknown cells beyond
        # it that border free cells inside -- a textbook frontier that no path
        # can ever reach. Measured on depot 2026-08-24: 52% of all goals failed,
        # and 43% of leo2's failures were for frontiers at x>7 in a world that
        # ends at x=7. "xmin,xmax,ymin,ymax" in the rover's own map frame;
        # empty disables the check.
        self.declare_parameter('world_bounds', '')
        # Reject frontiers this close to the edge of the occupancy grid, where
        # "unknown" means "the grid ran out", not "unexplored".
        self.declare_parameter('frontier_border_margin', 3)   # cells
        # A frontier hugging a wall is unreachable even when it is inside the
        # world: the rover cannot put its centre within robot_radius of it.
        self.declare_parameter('frontier_min_clearance', 0.30)  # metres
        # --- multi-robot coordination ---
        # 'coordinated' = distributed greedy allocation w/ proximity discount;
        # 'independent' = uncoordinated baseline (ignore peers).
        self.declare_parameter('coordination_mode', 'coordinated')
        self.declare_parameter('peer_names', '')     # comma-sep, e.g. "leo2"
        # The frame both rovers' positions are compared in. With
        # multirobot_map_merge this was the global 'map'. Under tag
        # alignment there is no global map frame -- leo1/map is the shared
        # frame and alignment_tf_bridge connects leo2/map to it once the
        # estimate is trusted.
        self.declare_parameter('common_frame', 'map')
        # The alignment estimate moves as it converges, so a peer offset
        # looked up once and cached forever freezes coordination at the
        # first and worst transform. Re-resolve this often (seconds).
        self.declare_parameter('offset_refresh_sec', 5.0)
        # Merged (shared) occupancy grid, published in the common frame once
        # the alignment locks. When fresh, unknown cells of the OWN map that
        # the shared map already knows are treated as covered, so frontier
        # detection stops proposing space the peer has finished. Empty topic
        # disables the feature; a stale shared map (no message for
        # shared_map_max_age) silently degrades to own-map behaviour.
        self.declare_parameter('shared_map_topic', '')
        self.declare_parameter('shared_map_max_age', 20.0)
        self.declare_parameter('shared_alignment_topic', '')
        self.declare_parameter('cancel_redundant_shared_goal', True)
        self.declare_parameter('discount_radius', 3.0)      # meters
        self.declare_parameter('discount_strength', 1.0)    # 0..1
        # Don't declare exploration finished until we've actually driven this
        # far - guards against the startup livelock where the first frontier
        # goal sits inside Nav2's goal tolerance, "succeeds" without motion,
        # and the tiny initial map is mistaken for a complete one.
        self.declare_parameter('min_explore_distance', 4.0)  # meters
        self.declare_parameter('idle_cycles_to_finish', 3)
        self.declare_parameter('watchdog_period', 10.0)
        self.declare_parameter('return_to_init', True)
        self.declare_parameter('map_save_path', '/ros2_ws/maps/explored_map')
        # --- camera coverage / item search ---
        self.declare_parameter('camera_fov', 1.047)
        self.declare_parameter('detection_range', 3.0)
        self.declare_parameter('camera_coverage_target', 0.90)
        self.declare_parameter('sweep_min_cluster', 5)      # wall cells
        self.declare_parameter('sweep_max_cluster', 60)     # wall cells
        self.declare_parameter('coverage_update_period', 0.5)
        self.declare_parameter('detections_topic', '/leo1/aruco_detections')
        # Persistent, detector-confirmed landmarks in this rover's map frame.
        # Frontiers already border unknown cells, so a bonus near these points
        # expands marker-bearing areas early without commanding the rover to
        # drive directly at a wall-mounted marker.
        self.declare_parameter('landmarks_topic', '/leo1/aruco_markers')
        self.declare_parameter('marker_frontier_radius', 4.0)
        self.declare_parameter('marker_frontier_bonus', 18.0)
        self.declare_parameter('verify_min_views', 2)
        self.declare_parameter('verify_max_attempts', 2)
        self.declare_parameter('verify_min_view_distance', 0.5)
        self.declare_parameter('verify_min_view_interval', 5.0)
        self.declare_parameter('verify_min_yaw_delta', 0.35)
        self.declare_parameter('sweep_heading_weight', 0.75)
        self.declare_parameter('cmd_vel_topic', '/leo1/cmd_vel')
        self.declare_parameter('escape_after_skips', 6)
        # --- multi-robot item search (PR: shared registry + coverage) ---
        # share_claims=True consumes peer item confirmations and camera-
        # coverage claims (the "coordinated" item-search condition). Claims
        # are always *published* regardless, so recorders see every robot.
        self.declare_parameter('share_claims', False)
        self.declare_parameter('item_claims_topic', '/item_claims')
        self.declare_parameter('coverage_claims_topic',
                               '/camera_coverage_claims')
        self.declare_parameter('coverage_share_period', 3.0)

        gp = lambda n: self.get_parameter(n).value
        self.robot_name = gp('robot_name')
        self.base_frame = gp('robot_base_frame')
        self.camera_frame = gp('camera_frame')
        self.map_frame = gp('map_frame')
        self.planner_period = 1.0 / gp('planner_frequency')
        self.min_frontier_size = gp('min_frontier_size')
        self.potential_scale = gp('potential_scale')
        self.gain_scale = gp('gain_scale')
        self.progress_timeout = gp('progress_timeout')
        self.blacklist_radius = gp('blacklist_radius')
        self.blacklist_growth = gp('blacklist_growth')
        self.blacklist_max_radius = gp('blacklist_max_radius')
        self.blacklist_ttl = gp('blacklist_ttl')
        self.blacklist_long_ttl = gp('blacklist_long_ttl')
        self.blacklist_max_strikes = gp('blacklist_max_strikes')
        self.replan_distance = gp('replan_distance')
        self.goal_commit_time = gp('goal_commit_time')
        self.switch_margin = gp('switch_margin')
        self.validate_goals = gp('validate_goals')
        self.skip_ttl = gp('skip_ttl')
        self.claim_radius = gp('claim_radius')
        bounds_raw = str(gp('world_bounds') or '').strip()
        self.world_bounds = None
        if bounds_raw:
            try:
                vals = [float(v) for v in bounds_raw.replace(',', ' ').split()]
                if len(vals) == 4:
                    self.world_bounds = tuple(vals)
            except ValueError:
                pass
        self.frontier_border_margin = int(gp('frontier_border_margin'))
        self.frontier_min_clearance = float(gp('frontier_min_clearance'))
        self.coordination_mode = gp('coordination_mode')
        self.peer_names = [p for p in gp('peer_names').split(',') if p]
        self.common_frame = gp('common_frame')
        self.offset_refresh_sec = gp('offset_refresh_sec')
        self.discount_radius = gp('discount_radius')
        self.discount_strength = gp('discount_strength')
        self.min_explore_distance = gp('min_explore_distance')
        self.idle_cycles_to_finish = gp('idle_cycles_to_finish')
        self.watchdog_period = gp('watchdog_period')
        self.return_to_init = gp('return_to_init')
        self.map_save_path = gp('map_save_path')
        self.coverage_target = gp('camera_coverage_target')
        self.sweep_min_cluster = gp('sweep_min_cluster')
        self.sweep_max_cluster = gp('sweep_max_cluster')
        self.verify_min_views = gp('verify_min_views')
        self.verify_max_attempts = gp('verify_max_attempts')
        self.verify_min_view_distance = gp('verify_min_view_distance')
        self.verify_min_view_interval = gp('verify_min_view_interval')
        self.verify_min_yaw_delta = gp('verify_min_yaw_delta')
        self.sweep_heading_weight = gp('sweep_heading_weight')
        self.marker_frontier_radius = float(gp('marker_frontier_radius'))
        self.marker_frontier_bonus = float(gp('marker_frontier_bonus'))
        self.share_claims = gp('share_claims')
        self._peer_offsets = {}      # peer -> (offset, resolved_at_sec)
        self._peer_positions = {}    # peer -> last position in our map frame
        self._peer_tracking_started_at = None

        self.map_msg = None
        self.state = STATE_EXPLORING
        self.blacklist = []          # [{'pos','expires','strikes'}]
        self.skip_list = []          # [{'pos','expires'}] planner said no path
        self.peer_claims = {}        # robot_name -> (x, y) in OUR map frame
        self._common_offset = None   # (offset, resolved_at_sec)
        self.init_pose = None
        self.idle_cycles = 0

        self.goal_handle = None
        self.goal_pos = None
        self.goal_kind = GOAL_FRONTIER
        self.goal_score = 0.0
        self.goal_sent_time = None
        self.navigating = False
        self.validation_in_flight = False
        self.best_dist_to_goal = None
        self.last_progress_time = None
        self.cleared_costmaps_for_goal = False

        # item registry: id -> {'pos','z','views','confirmed','attempts'}
        self.items = {}
        self.landmarks = {}          # confirmed ArUco id -> own-map (x, y)
        self._marker_priority_frontier_count = 0
        self.pending_verify = deque()
        self.verify_target = None
        self.last_sweep_target = None
        self.last_done_frontier = None

        self.coverage = CameraCoverageTracker(
            fov=gp('camera_fov'), detection_range=gp('detection_range'))
        self._coverage_frac = 0.0
        self._coverage_counts = (0, 0)
        self._cov_tick = 0

        self.pos_history = deque(maxlen=64)   # (t_sec, x, y)
        self.skips_since_dispatch = 0
        self.escape_ticks = 0
        self.escape_count = 0
        self.escape_after_skips = gp('escape_after_skips')
        self.stats = {
            'goals_sent': 0, 'goals_succeeded': 0, 'goals_failed': 0,
            'validations_rejected': 0, 'costmap_clears': 0,
            'watchdog_recoveries': 0, 'escapes': 0, 'distance_traveled': 0.0,
            'sweep_goals': 0, 'verify_goals': 0,
            'peer_goals_cancelled': 0,
            'marker_priority_goals': 0,
        }
        self._last_xy = None
        self._last_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, gp('map_topic'), self._map_cb, map_qos)
        self.create_subscription(
            MarkerArray, gp('detections_topic'), self._detection_cb, 10)
        self.create_subscription(
            MarkerArray, gp('landmarks_topic'), self._landmarks_cb, map_qos)

        # Shared (merged) map. The merger publishes VOLATILE; requesting
        # TRANSIENT_LOCAL here would silently never match (same QoS mismatch
        # class as the velocity_guard /wheel_odom bug).
        self.shared_map_msg = None
        self._shared_map_time = None
        self._shared_mask_active = False
        self._shared_map_available = False
        self._shared_masked_cells = 0
        shared_topic = str(gp('shared_map_topic')).strip()
        self.shared_map_max_age = float(gp('shared_map_max_age'))
        self.cancel_redundant_shared_goal = bool(
            gp('cancel_redundant_shared_goal'))
        self._shared_map_configured = bool(shared_topic)
        self._completion_basis = 'local map'
        self._waiting_for_shared_completion = False
        alignment_topic = str(gp('shared_alignment_topic')).strip()
        self._shared_alignment_required = bool(alignment_topic)
        self._shared_alignment_locked = not self._shared_alignment_required
        if alignment_topic:
            self.create_subscription(
                Bool, alignment_topic, self._shared_alignment_cb, 10)
        if shared_topic:
            shared_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                OccupancyGrid, shared_topic, self._shared_map_cb, shared_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(
            self, ComputePathToPose, gp('planner_action_name'))

        # Relative names so they resolve under the node's namespace (per-robot
        # /leo{i}/... in multi-robot runs, /... in single-robot runs).
        self.clear_local_client = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.clear_global_client = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')

        self.claim_pub = self.create_publisher(Marker, '/exploration_claims', 10)
        self.create_subscription(
            Marker, '/exploration_claims', self._claim_cb, 10)
        self.frontier_pub = self.create_publisher(MarkerArray, '~/frontiers', 10)
        self.status_pub = self.create_publisher(StringMsg, '~/status', 10)
        self.coverage_pub = self.create_publisher(
            OccupancyGrid, '~/camera_coverage', 1)
        self.items_pub = self.create_publisher(StringMsg, '~/found_items', 10)
        self.items_marker_pub = self.create_publisher(
            MarkerArray, '~/found_items_markers', 10)

        # Shared item registry + camera-coverage claims (global topics,
        # positions/keys in the common 'map' frame).
        self.item_claims_pub = self.create_publisher(
            StringMsg, gp('item_claims_topic'), 10)
        self.create_subscription(
            StringMsg, gp('item_claims_topic'), self._item_claims_cb, 10)
        self.coverage_claims_pub = self.create_publisher(
            StringMsg, gp('coverage_claims_topic'), 10)
        self.create_subscription(
            StringMsg, gp('coverage_claims_topic'),
            self._coverage_claims_cb, 10)
        self.create_timer(gp('coverage_share_period'), self._share_coverage)

        self.save_map_client = self.create_client(
            SaveMap, 'slam_toolbox/save_map')
        self.serialize_client = self.create_client(
            SerializePoseGraph, 'slam_toolbox/serialize_map')

        self.cmd_vel_pub = self.create_publisher(
            Twist, gp('cmd_vel_topic'), 10)

        self.create_timer(self.planner_period, self._plan_cycle)
        self.create_timer(self.watchdog_period, self._watchdog)
        self.create_timer(0.2, self._escape_tick)
        self.create_timer(gp('coverage_update_period'), self._coverage_tick)
        self.get_logger().info(
            f'Frontier explorer up for "{self.robot_name}" '
            f'(base_frame={self.base_frame}); coordination='
            f'{self.coordination_mode}, peers={self.peer_names or "none"}')

    # ------------------------------------------------------------- callbacks

    def _map_cb(self, msg):
        self.map_msg = msg

    def _shared_map_cb(self, msg):
        self.shared_map_msg = msg
        self._shared_map_time = self._now_sec()

    def _shared_alignment_cb(self, msg):
        locked = bool(msg.data)
        if self._shared_alignment_locked and not locked:
            self.get_logger().warn(
                'shared-map alignment lock lost; disabling peer frontier mask')
        self._shared_alignment_locked = locked
        if not locked:
            self._shared_map_available = False
            self._shared_mask_active = False
            self._shared_masked_cells = 0

    def _claim_cb(self, msg):
        if msg.ns == self.robot_name:
            return
        if msg.action == Marker.DELETE:
            self.peer_claims.pop(msg.ns, None)
        else:
            # Claims are shared in the common 'map' frame; convert into our own
            # map frame so the allocation compares them against our frontiers.
            mine = self._from_common((msg.pose.position.x, msg.pose.position.y))
            if mine is None:
                # No shared frame yet, so a peer claim cannot be placed on our
                # map at all. Dropping it is right: before this, the raw
                # coordinates were stored as if they were already ours.
                return
            self.peer_claims[msg.ns] = mine

    def _detection_cb(self, msg):
        view = self._camera_view()
        for det in msg.markers:
            mid = int(det.id)
            p = det.pose.position
            item = self.items.get(mid)
            if item is None:
                self.items[mid] = {
                    'pos': (p.x, p.y), 'z': p.z, 'views': 1,
                    'confirmed': False, 'attempts': 0, 'last_view': view,
                }
                self.pending_verify.append(mid)
                self.get_logger().info(
                    f'Item candidate id={mid} at ({p.x:.2f}, {p.y:.2f}) - '
                    f'queued for verification')
            else:
                if not distinct_view(
                        item.get('last_view'), view,
                        self.verify_min_view_distance,
                        self.verify_min_view_interval,
                        self.verify_min_yaw_delta):
                    continue
                n = item['views']
                item['pos'] = ((item['pos'][0] * n + p.x) / (n + 1),
                               (item['pos'][1] * n + p.y) / (n + 1))
                item['views'] = n + 1
                item['last_view'] = view
                if not item['confirmed'] \
                        and item['views'] >= self.verify_min_views:
                    item['confirmed'] = True
                    self.get_logger().info(
                        f'Item id={mid} CONFIRMED at '
                        f'({item["pos"][0]:.2f}, {item["pos"][1]:.2f}) '
                        f'after {item["views"]} views')
                    if self.verify_target == mid:
                        self.verify_target = None
                        self._cancel_goal()
        self._publish_items()

    def _landmarks_cb(self, msg):
        """Replace the local copy of the detector's persistent registry."""
        current = {}
        expected = str(self.map_frame).lstrip('/')
        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue
            frame = str(marker.header.frame_id).lstrip('/')
            if frame and frame != expected:
                continue
            current[int(marker.id)] = (
                float(marker.pose.position.x),
                float(marker.pose.position.y),
            )
        self.landmarks = current

    # ------------------------------------------- shared item/coverage claims

    def _get_peer_offset(self, peer):
        """Peer's map frame origin in the common frame.

        Refreshed rather than cached for the life of the run: before the first
        mutual tag sighting there is no such transform at all (we return None
        and the caller degrades to working alone), and afterwards the estimate
        keeps improving as more common landmarks are seen. Pinning the first
        value would lock coordination to the worst transform of the run.
        """
        cached = self._peer_offsets.get(peer)
        now = self._now_sec()
        if cached is not None and now - cached[1] < self.offset_refresh_sec:
            return cached[0]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.common_frame, f'{peer}/map', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            # Keep serving the last good value until it ages out entirely, so
            # a momentary lookup failure does not drop us out of coordination.
            return cached[0] if cached is not None else None
        offset = (tf.transform.translation.x, tf.transform.translation.y)
        self._peer_offsets[peer] = (offset, now)
        return offset

    def _publish_item_claims(self):
        """Broadcast the registry in the common frame for peers/recorders."""
        payload = {
            'robot': self.robot_name,
            'sim_time': round(self._now_sec(), 1),
            'items': [
                {'id': mid, 'x': round((self._to_common(it['pos']) or it['pos'])[0], 3),
                 'y': round((self._to_common(it['pos']) or it['pos'])[1], 3),
                 'views': it['views'], 'confirmed': it['confirmed'],
                 'via_peer': it.get('via_peer', False)}
                for mid, it in sorted(self.items.items())
            ],
        }
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self.item_claims_pub.publish(msg)

    def _item_claims_cb(self, msg):
        """Adopt peer-CONFIRMED items so we never drive a redundant verify.
        Peer candidates are ignored (each robot verifies its own sightings)."""
        if not self.share_claims:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, KeyError):
            return
        if payload.get('robot') == self.robot_name:
            return
        off = self._get_common_offset()
        if off is None:
            return
        changed = False
        for it in payload.get('items', []):
            if not it.get('confirmed'):
                continue
            mid = int(it['id'])
            own = self.items.get(mid)
            pos_own = self._from_common((it['x'], it['y']))
            if pos_own is None:
                continue
            if own is None:
                self.items[mid] = {
                    'pos': pos_own, 'z': 0.3, 'views': it.get('views', 2),
                    'confirmed': True, 'attempts': 0, 'last_view': None,
                    'via_peer': True,
                }
            elif not own['confirmed']:
                own['confirmed'] = True
                own['via_peer'] = True
            else:
                continue
            changed = True
            if mid in self.pending_verify:
                self.pending_verify = deque(
                    m for m in self.pending_verify if m != mid)
            if self.verify_target == mid:
                self.verify_target = None
                self._cancel_goal()
        if changed:
            self.get_logger().info(
                f'Adopted peer-confirmed items from '
                f'{payload.get("robot")}: now '
                f'{sum(1 for i in self.items.values() if i["confirmed"])} '
                f'confirmed')
            self._publish_items()

    def _share_coverage(self):
        """Broadcast observed wall keys (world-quantized, common frame)."""
        if not self.coverage.observed or self.map_msg is None:
            return
        res = self.map_msg.info.resolution
        # Camera-coverage cells are shared as common-frame grid indices, so
        # each one has to be rotated, not just shifted.
        keys = []
        for i, j in self.coverage.observed:
            pt = self._to_common((i * res, j * res))
            if pt is None:
                return
            keys.append([int(round(pt[0] / res)), int(round(pt[1] / res))])
        msg = StringMsg()
        msg.data = json.dumps({'robot': self.robot_name, 'res': res,
                               'keys': keys})
        self.coverage_claims_pub.publish(msg)

    def _coverage_claims_cb(self, msg):
        """Import a peer's observed wall cells so we do not re-sweep them."""
        if not self.share_claims or self.map_msg is None:
            return
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if payload.get('robot') == self.robot_name:
            return
        res = self.map_msg.info.resolution
        if abs(payload.get('res', res) - res) > 1e-6:
            return
        if self._get_common_offset() is None:
            return
        for i, j in payload.get('keys', []):
            pt = self._from_common((int(i) * res, int(j) * res))
            if pt is None:
                return
            self.coverage.observed.add(
                (int(round(pt[0] / res)), int(round(pt[1] / res))))

    # ----------------------------------------------------------- robot pose

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _camera_view(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        p = tf.transform.translation
        return (self._now_sec(), p.x, p.y,
                yaw_from_quat(tf.transform.rotation))

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        pos = tf.transform.translation
        self._last_yaw = yaw_from_quat(tf.transform.rotation)
        if self.init_pose is None:
            self.init_pose = (pos.x, pos.y)
            self.get_logger().info(
                f'Initial pose recorded: ({pos.x:.2f}, {pos.y:.2f})')
        self.pos_history.append((self._now_sec(), pos.x, pos.y))
        if self._last_xy is not None:
            self.stats['distance_traveled'] += math.hypot(
                pos.x - self._last_xy[0], pos.y - self._last_xy[1])
        self._last_xy = (pos.x, pos.y)
        return pos.x, pos.y

    # ------------------------------------------------------ camera coverage

    def _coverage_tick(self):
        if self.map_msg is None or self.state in (STATE_SAVING, STATE_DONE):
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        self.coverage.update(self.map_msg, t.x, t.y, yaw)
        self._cov_tick += 1
        if self._cov_tick % 10 == 0:
            observed, total, frac = self.coverage.stats(self.map_msg)
            self._coverage_frac = frac
            self._coverage_counts = (observed, total)
            self.coverage_pub.publish(
                self.coverage.grid_msg(self.map_msg, self.map_frame))

    # ------------------------------------------------------------ frontiers

    def _find_frontiers(self):
        """Return list of clusters: dicts with size_m, centroid, goal."""
        msg = self.map_msg
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            info.height, info.width)

        free = (grid >= 0) & (grid < FREE_MAX)
        unknown = grid == UNKNOWN

        # Peer-covered space is covered space: unknown cells of OUR map that
        # the merged map already knows stop generating frontiers, so the
        # rover does not drive across a room the peer finished. Applied to
        # the unknown mask, NOT the frontier cells -- a frontier cell itself
        # is free in the merged map by construction (our own map is merged
        # in), so masking frontier cells would erase every frontier.
        # Degrades seamlessly: no shared map, stale shared map, or no
        # alignment TF -> mask is None -> own-map behaviour, no stalling.
        peer_known = self._peer_known_mask(info, unknown)
        if peer_known is not None:
            unknown &= ~peer_known

        near_unknown = np.zeros_like(unknown)
        near_unknown[1:, :] |= unknown[:-1, :]
        near_unknown[:-1, :] |= unknown[1:, :]
        near_unknown[:, 1:] |= unknown[:, :-1]
        near_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & near_unknown

        # Drop frontier cells at the very edge of the grid: there "unknown"
        # means the array ran out, not that there is anything to go and see.
        m = self.frontier_border_margin
        if m > 0 and frontier.shape[0] > 2 * m and frontier.shape[1] > 2 * m:
            edge = np.ones_like(frontier)
            edge[m:-m, m:-m] = False
            frontier &= ~edge

        # Drop frontier cells that hug an obstacle. The rover's centre can
        # never get within robot_radius of a wall, so a frontier pressed
        # against one is unreachable however valid it looks on the grid.
        clear_cells = int(round(self.frontier_min_clearance / info.resolution))
        if clear_cells > 0:
            occupied = grid >= FREE_MAX
            near_occ = occupied.copy()
            for _ in range(clear_cells):
                grown = near_occ.copy()
                grown[1:, :] |= near_occ[:-1, :]
                grown[:-1, :] |= near_occ[1:, :]
                grown[:, 1:] |= near_occ[:, :-1]
                grown[:, :-1] |= near_occ[:, 1:]
                near_occ = grown
            frontier &= ~near_occ

        # And drop anything outside the world we were told about.
        if self.world_bounds is not None:
            xmin, xmax, ymin, ymax = self.world_bounds
            rows, cols = np.indices(frontier.shape)
            wx, wy = FrontierExplorer._grid_cells_to_world(info, rows, cols)
            frontier &= (wx >= xmin) & (wx <= xmax) & (wy >= ymin) & (wy <= ymax)

        cells = np.argwhere(frontier)
        if cells.size == 0:
            return []

        frontier_set = {(int(r), int(c)) for r, c in cells}
        res = info.resolution
        clusters = []
        while frontier_set:
            seed = frontier_set.pop()
            queue = deque([seed])
            members = [seed]
            while queue:
                r, c = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in frontier_set:
                            frontier_set.remove(n)
                            queue.append(n)
                            members.append(n)
            pts = np.array(members, dtype=float)
            cy, cx = pts.mean(axis=0)
            d2 = ((pts[:, 0] - cy) ** 2 + (pts[:, 1] - cx) ** 2)
            gr, gc = pts[int(np.argmin(d2))]
            gr, gc = self._snap_goal(free, int(gr), int(gc))
            centroid_x, centroid_y = FrontierExplorer._grid_cells_to_world(
                info, np.asarray(cy), np.asarray(cx))
            goal_x, goal_y = FrontierExplorer._grid_cells_to_world(
                info, np.asarray(gr), np.asarray(gc))
            clusters.append({
                'size_m': len(members) * res,
                'centroid': (float(centroid_x), float(centroid_y)),
                'goal': (float(goal_x), float(goal_y)),
            })
        return clusters

    @staticmethod
    def _origin_yaw(info):
        q = getattr(info.origin, 'orientation', None)
        if q is None:
            return 0.0
        return math.atan2(
            2.0 * (getattr(q, 'w', 1.0) * getattr(q, 'z', 0.0)
                   + getattr(q, 'x', 0.0) * getattr(q, 'y', 0.0)),
            1.0 - 2.0 * (getattr(q, 'y', 0.0) ** 2
                         + getattr(q, 'z', 0.0) ** 2),
        )

    @staticmethod
    def _grid_cells_to_world(info, rows, cols):
        local_x = (cols + 0.5) * info.resolution
        local_y = (rows + 0.5) * info.resolution
        yaw = FrontierExplorer._origin_yaw(info)
        c, s = math.cos(yaw), math.sin(yaw)
        return (
            info.origin.position.x + c * local_x - s * local_y,
            info.origin.position.y + s * local_x + c * local_y,
        )

    def _peer_known_mask(self, info, unknown):
        """Bool array marking OUR unknown cells the shared map knows, or None.

        None (= no masking) whenever the shared map is absent or stale, or
        the own-frame -> common-frame offset is unavailable (alignment not
        locked). The mask is recomputed per planning cycle because both the
        shared map and the alignment estimate move.
        """
        msg = self.shared_map_msg
        if msg is None or self._shared_map_time is None:
            return None
        # The central merger publishes a leo1-only fallback before alignment.
        # It may also clean tiny unknown holes. Neither is peer coverage.
        # Never let that pre-lock map cancel or mask a real frontier.
        if (self._shared_alignment_required
                and not self._shared_alignment_locked):
            self._shared_map_available = False
            self._shared_mask_active = False
            self._shared_masked_cells = 0
            return None
        age = self._now_sec() - self._shared_map_time
        if age > self.shared_map_max_age:
            if self._shared_map_available:
                self._shared_mask_active = False
                self._shared_map_available = False
                self.get_logger().info(
                    f'shared-map mask OFF (stale by {age:.0f}s); '
                    'falling back to own map')
            return None
        # The grid says which frame it lives in. A per-rover merger (Phase 3)
        # publishes in the rover's OWN map frame -> identity, no TF needed,
        # so a dead laptop cannot take the mask down with it. The central
        # merger publishes in the common frame -> alignment TF lookup.
        frame = msg.header.frame_id or self.common_frame
        if frame == self.map_frame:
            off = (0.0, 0.0, 0.0)
        else:
            off = self._get_common_offset()
        if off is None:
            return None
        rows, cols = np.nonzero(unknown)
        if not self._shared_map_available:
            self.get_logger().info(
                'shared-map union available: peer-covered unknown space now '
                'suppresses local frontier work')
        self._shared_map_available = True
        if rows.size == 0:
            self._shared_masked_cells = 0
            return np.zeros_like(unknown)
        wx, wy = FrontierExplorer._grid_cells_to_world(info, rows, cols)
        c, s = math.cos(off[2]), math.sin(off[2])
        gx = off[0] + c * wx - s * wy
        gy = off[1] + s * wx + c * wy
        sinfo = msg.info
        sgrid = np.asarray(msg.data, dtype=np.int8).reshape(
            sinfo.height, sinfo.width)
        # OccupancyGrid origin orientation is part of the map geometry. Undo
        # it before indexing the shared grid (real SLAM maps are allowed to
        # publish a non-identity origin pose).
        dx = gx - sinfo.origin.position.x
        dy = gy - sinfo.origin.position.y
        syaw = FrontierExplorer._origin_yaw(sinfo)
        sc, ss = math.cos(syaw), math.sin(syaw)
        local_x = sc * dx + ss * dy
        local_y = -ss * dx + sc * dy
        ci = np.floor(local_x / sinfo.resolution).astype(int)
        ri = np.floor(local_y / sinfo.resolution).astype(int)
        ok = ((ci >= 0) & (ci < sinfo.width) & (ri >= 0) & (ri < sinfo.height))
        known = np.zeros(rows.size, dtype=bool)
        known[ok] = sgrid[ri[ok], ci[ok]] != UNKNOWN
        mask = np.zeros_like(unknown)
        mask[rows[known], cols[known]] = True
        n = int(known.sum())
        self._shared_masked_cells = n
        active_before = self._shared_mask_active
        self._shared_mask_active = n > 0
        if n > 0 and not active_before:
            self._shared_mask_active = True
            self.get_logger().info(
                f'shared-map mask ON: {n} unknown cells already covered '
                'by the merged map')
        return mask

    @staticmethod
    def _snap_goal(free, r, c, radius=8):
        """Walk the goal to the nearest cell whose 3x3 block is all free."""
        h, w = free.shape

        def solid(rr, cc):
            if rr < 1 or cc < 1 or rr >= h - 1 or cc >= w - 1:
                return False
            return bool(free[rr - 1:rr + 2, cc - 1:cc + 2].all())

        if solid(r, c):
            return r, c
        for rad in range(1, radius + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    if max(abs(dr), abs(dc)) != rad:
                        continue
                    if solid(r + dr, c + dc):
                        return r + dr, c + dc
        return r, c

    def _prune_lists(self):
        # Expired blacklist entries stop *blocking* but are kept as strike
        # memory for a while - otherwise a goal that fails every ~TTL never
        # accumulates strikes and is retried forever.
        now = self._now_sec()
        memory = self.blacklist_ttl * 3
        self.blacklist = [b for b in self.blacklist
                          if b['expires'] + memory > now]
        # Same memory trick for skips: an expired entry stops blocking but
        # keeps its strike count so a later rejection escalates instead of
        # starting over. Ghosts are typically re-attempted only every few
        # minutes (the robot works other goals in between), so the memory
        # must be generous.
        self.skip_list = [s for s in self.skip_list
                          if s['expires'] + 600.0 > now]

    def _entry_radius(self, e):
        """Banned radius for one entry: grows with repeat strikes."""
        extra = self.blacklist_growth * max(0, int(e.get('strikes', 1)) - 1)
        return min(self.blacklist_radius + extra, self.blacklist_max_radius)

    def _is_blocked(self, xy, entries):
        now = self._now_sec()
        return any(e.get('expires', 0) > now
                   and math.hypot(xy[0] - e['pos'][0], xy[1] - e['pos'][1])
                   < self._entry_radius(e) for e in entries)

    def _add_blacklist(self, xy, strikes=1):
        now = self._now_sec()
        for e in self.blacklist:
            if math.hypot(xy[0] - e['pos'][0], xy[1] - e['pos'][1]) \
                    < self.blacklist_radius:
                e['strikes'] = max(e['strikes'] + 1, strikes)
                ttl = (self.blacklist_long_ttl
                       if e['strikes'] >= self.blacklist_max_strikes
                       else self.blacklist_ttl)
                e['expires'] = now + ttl
                return
        ttl = (self.blacklist_long_ttl
               if strikes >= self.blacklist_max_strikes
               else self.blacklist_ttl)
        self.blacklist.append(
            {'pos': xy, 'expires': now + ttl, 'strikes': strikes})

    def _add_skip(self, xy):
        # Repeated planner rejections of the same spot escalate into the
        # blacklist: gaps the lidar sees through but the planner can never
        # reach ("ghost frontiers") would otherwise cycle off the 20 s skip
        # TTL forever and starve the sweep phase.
        now = self._now_sec()
        for s in self.skip_list:
            if math.hypot(xy[0] - s['pos'][0], xy[1] - s['pos'][1]) \
                    < self.blacklist_radius:
                s['strikes'] = s.get('strikes', 1) + 1
                s['expires'] = now + self.skip_ttl
                if s['strikes'] >= 3:
                    # Three planner rejections == confirmed unreachable.
                    # Enter the blacklist at max strikes so the long ban
                    # applies immediately and the entry survives watchdog
                    # blacklist resets - a fresh 1-strike entry gets wiped
                    # by the reset before it can ever mature, which is
                    # exactly the ghost-grind livelock.
                    self._add_blacklist(xy, strikes=self.blacklist_max_strikes)
                    self.get_logger().warn(
                        f'Goal ({xy[0]:.2f}, {xy[1]:.2f}) planner-rejected '
                        f'{s["strikes"]} times - long-banning')
                return
        self.skip_list.append(
            {'pos': xy, 'expires': now + self.skip_ttl, 'strikes': 1})

    def _eligible(self, cluster):
        if cluster['size_m'] < self.min_frontier_size:
            return False
        gx, gy = cluster['goal']
        if self._is_blocked((gx, gy), self.blacklist):
            return False
        if self._is_blocked((gx, gy), self.skip_list):
            return False
        # Peer coordination is handled by the allocation policy in
        # _select_frontier (which load-balances and fans robots apart), not by
        # excluding frontiers here - excluding them would starve the allocator.
        return True

    def _score(self, cluster, robot):
        return (self.gain_scale * cluster['size_m']
                - self.potential_scale * math.hypot(
                    cluster['goal'][0] - robot[0],
                    cluster['goal'][1] - robot[1])
                + self._marker_bonus(cluster))

    def _marker_bonus(self, cluster):
        """Return a distance-decaying bonus for marker-adjacent unknown."""
        if not self.landmarks or self.marker_frontier_radius <= 0.0:
            return 0.0
        gx, gy = cluster['goal']
        distance = min(
            math.hypot(gx - marker[0], gy - marker[1])
            for marker in self.landmarks.values())
        if distance >= self.marker_frontier_radius:
            return 0.0
        return self.marker_frontier_bonus * (
            1.0 - distance / self.marker_frontier_radius)

    @staticmethod
    def _goal_has_matching_frontier(goal, clusters, replan_distance):
        if goal is None:
            return False
        return any(math.hypot(
            cluster['goal'][0] - goal[0],
            cluster['goal'][1] - goal[1]) < replan_distance
            for cluster in clusters)

    # ----------------------------------------------------------- main cycle

    def _plan_cycle(self):
        if self.state in (STATE_SAVING, STATE_DONE, STATE_RETURNING):
            return
        if self.map_msg is None:
            return
        robot = self._robot_xy()
        if robot is None:
            return

        # SLAM warmup: until the map contains walls, slam_toolbox is still
        # waiting for the robot to move (minimum_travel_distance/heading).
        # Nav goals this early "succeed" without motion, deadlocking startup.
        # Spin in place to feed scan matching until occupied cells appear.
        grid = np.asarray(self.map_msg.data, dtype=np.int8)
        if (grid >= FREE_MAX).sum() < 20:
            if self.escape_ticks == 0:
                self.get_logger().info(
                    'Map has (almost) no walls yet - warmup drive for SLAM')
                # slam_toolbox only starts integrating after *translation*
                # exceeds minimum_travel_distance; rotation alone is not
                # enough. Drive a slow straight segment (linear branch).
                self.escape_count = 1
                self.escape_ticks = 25  # 5 s at 0.15 m/s = 0.75 m
            return

        self._prune_lists()

        # Priority 1: verify item candidates with a dedicated viewpoint goal.
        if self._handle_verification(robot):
            return

        clusters = [c for c in self._find_frontiers() if self._eligible(c)]
        self._marker_priority_frontier_count = sum(
            1 for cluster in clusters if self._marker_bonus(cluster) > 0.0)

        # Re-evaluate an in-flight frontier against the latest shared union.
        # A goal was valid when dispatched can disappear after the other rover
        # maps the room. Cancel it immediately instead of honoring commitment
        # hysteresis and duplicating the peer's travel.
        if (self.cancel_redundant_shared_goal
                and self._shared_map_available
                and self._shared_mask_active
                and self.navigating
                and self.goal_kind == GOAL_FRONTIER
                and self.goal_pos is not None
                and not FrontierExplorer._goal_has_matching_frontier(
                    self.goal_pos, clusters, self.replan_distance)):
            old_goal = self.goal_pos
            self.stats['peer_goals_cancelled'] += 1
            self.get_logger().info(
                f'CANCELLED REDUNDANT FRONTIER ({old_goal[0]:.2f}, '
                f'{old_goal[1]:.2f}): shared map shows the peer already '
                'covered its unknown region')
            self._cancel_goal()

        self._publish_frontier_markers(clusters)
        self._publish_status(len(clusters))

        # A frontier goal can "succeed" without revealing any map: SLAM
        # drift sometimes opens a sliver of unknown behind a wall that the
        # lidar can never see from the reachable side. Visiting it again is
        # pointless - if the frontier survived a *succeeded* visit in place,
        # strike it (same cure as the sweep-target strike).
        if self.last_done_frontier is not None and not self.navigating:
            for c in clusters:
                if math.hypot(c['goal'][0] - self.last_done_frontier[0],
                              c['goal'][1] - self.last_done_frontier[1]) \
                        < self.blacklist_radius:
                    self._add_blacklist(c['goal'])
                    self.get_logger().warn(
                        f'Frontier goal done but frontier survived at '
                        f'({c["goal"][0]:.2f}, {c["goal"][1]:.2f}) - '
                        f'striking it')
            self.last_done_frontier = None
            clusters = [c for c in clusters if self._eligible(c)]

        if clusters:
            self.idle_cycles = 0
            if self.state != STATE_EXPLORING:
                self.get_logger().info('New frontiers - back to EXPLORING')
                self.state = STATE_EXPLORING
            self._frontier_step(clusters, robot)
            return

        # No frontiers left: sweep walls the camera has not seen yet.
        if self._sweep_step(robot):
            return

        self.idle_cycles += 1
        if (self.idle_cycles >= self.idle_cycles_to_finish
                and not self.navigating
                and not self.validation_in_flight
                and not self.pending_verify):
            # Startup-livelock guard: "out of frontiers" while barely moved
            # means the first goal was struck before revealing any map. Drive
            # a short segment to expose new frontiers instead of finishing.
            if (self.stats['distance_traveled'] < self.min_explore_distance
                    and self.escape_ticks == 0):
                self.get_logger().warn(
                    f'Idle at {self.stats["distance_traveled"]:.1f} m explored '
                    f'(< {self.min_explore_distance} m) - warmup drive to '
                    f'break startup livelock')
                self.escape_count += 1
                self.escape_ticks = 25
                self.idle_cycles = 0
                return
            shared_trusted = self._shared_map_available and (
                not self.peer_names
                or any(self._peer_xy(peer) is not None
                       for peer in self.peer_names)
            )
            if (self._shared_map_configured
                    and self.coordination_mode == 'coordinated'
                    and not shared_trusted):
                if not self._waiting_for_shared_completion:
                    self._waiting_for_shared_completion = True
                    self.get_logger().info(
                        'LOCAL FRONTIERS EXHAUSTED; holding stopped until the '
                        'vetted shared frame is available, then global '
                        'frontier completion will be checked')
                self.cmd_vel_pub.publish(Twist())
                return
            self._waiting_for_shared_completion = False
            if shared_trusted:
                self._completion_basis = 'vetted shared-map union'
                self.get_logger().info(
                    'GLOBAL SHARED COVERAGE COMPLETE for this rover: no '
                    'novel frontier remains after peer-map masking; stopping '
                    'instead of revisiting peer-explored rooms')
            else:
                self._completion_basis = 'local map (shared union unavailable)'
            self._finish_exploration()

    def _get_common_offset(self):
        """Our map frame's origin expressed in the common frame (x, y).

        Zero when we already navigate in the common frame -- which is the case
        for leo1 when the common frame is leo1/map. For leo2 this is the
        recovered alignment, so it is refreshed on the same schedule as
        _get_peer_offset rather than treated as static; under tag alignment it
        is not static, and it does not exist at all until the rovers have seen
        common landmarks.
        """
        if self.map_frame == self.common_frame:
            return (0.0, 0.0, 0.0)
        now = self._now_sec()
        if self._common_offset is not None                 and now - self._common_offset[1] < self.offset_refresh_sec:
            return self._common_offset[0]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.common_frame, self.map_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return self._common_offset[0] if self._common_offset else None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        offset = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        self._common_offset = (offset, now)
        return offset

    def _to_common(self, xy):
        """A point in our own map frame -> the common frame."""
        off = self._get_common_offset()
        if off is None or xy is None:
            return None
        c, s = math.cos(off[2]), math.sin(off[2])
        return (off[0] + c * xy[0] - s * xy[1],
                off[1] + s * xy[0] + c * xy[1])

    def _from_common(self, xy):
        """A point in the common frame -> our own map frame."""
        off = self._get_common_offset()
        if off is None or xy is None:
            return None
        dx, dy = xy[0] - off[0], xy[1] - off[1]
        c, s = math.cos(-off[2]), math.sin(-off[2])
        return (c * dx - s * dy, s * dx + c * dy)

    def _peer_xy(self, peer):
        """Peer rover position from the shared (merged) TF tree, or None."""
        base_suffix = self.base_frame.split('/', 1)[-1]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, f'{peer}/{base_suffix}', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        position = (tf.transform.translation.x, tf.transform.translation.y)
        self._peer_positions[peer] = position
        if self._peer_tracking_started_at is None:
            self._peer_tracking_started_at = self._now_sec()
            moving = self.navigating or abs(self.stats['distance_traveled']) > 0.05
            self.get_logger().info(
                'PEER POSITION TRACKING STARTED at '
                f't={self._peer_tracking_started_at:.1f}s: {peer} resolved in '
                f'{self.map_frame} at ({position[0]:.2f}, {position[1]:.2f}); '
                f'robot_in_motion={moving}. Coordinated frontier allocation '
                'is active from this cycle.')
        return position

    def _select_frontier(self, clusters, robot):
        """Pick this rover's frontier. In 'coordinated' mode run the shared
        greedy allocation over all rovers (poses from the merged TF tree, peer
        commitments from /exploration_claims) and take our own assignment; in
        'independent' mode just take the locally best-scoring frontier."""
        marker_frontiers = [
            cluster for cluster in clusters
            if self._marker_bonus(cluster) > 0.0
        ]
        # A peer claim remains authoritative: both rovers should not abandon
        # their allocation to chase the same marker-adjacent room.
        unclaimed_marker_frontiers = [
            cluster for cluster in marker_frontiers
            if not any(
                math.hypot(
                    cluster['goal'][0] - claim[0],
                    cluster['goal'][1] - claim[1],
                ) < self.discount_radius
                for claim in self.peer_claims.values()
            )
        ]

        # Commitment stickiness: if we are already driving to a frontier that
        # is still a valid frontier, keep it. Re-running the global allocation
        # every cycle otherwise makes a trailing rover thrash - it chases the
        # shared corridor as the leader moves and never commits the ~30 s
        # approach needed to actually turn into a room (observed on
        # office_world: leo2 logged 46 m of corridor travel, 21 room-goal
        # assignments, but never once left y~0).
        if (self.navigating and self.goal_kind == GOAL_FRONTIER
                and self.goal_pos is not None):
            for c in clusters:
                if math.hypot(c['goal'][0] - self.goal_pos[0],
                              c['goal'][1] - self.goal_pos[1]) \
                        < self.replan_distance:
                    # A marker first seen during travel is allowed to propose
                    # an earlier switch. _frontier_step still enforces commit
                    # time and score hysteresis before cancelling anything.
                    if (unclaimed_marker_frontiers
                            and self._marker_bonus(c) <= 0.0):
                        break
                    return c
        if unclaimed_marker_frontiers:
            return max(
                unclaimed_marker_frontiers,
                key=lambda cluster: self._score(cluster, robot))
        if self.coordination_mode == 'coordinated' and self.peer_names:
            robots = [(self.robot_name, robot)]
            for peer in self.peer_names:
                pxy = self._peer_xy(peer)
                if pxy is not None:
                    robots.append((peer, pxy))
            if len(robots) > 1:
                committed = {p: self.peer_claims[p] for p in self.peer_names
                             if p in self.peer_claims}
                assignment = coordinated_allocation(
                    robots, clusters, committed=committed,
                    potential_scale=self.potential_scale,
                    gain_scale=self.gain_scale,
                    discount_radius=self.discount_radius,
                    discount_strength=self.discount_strength)
                my_goal = assignment.get(self.robot_name)
                if my_goal is not None:
                    return min(clusters, key=lambda c: math.hypot(
                        c['goal'][0] - my_goal[0], c['goal'][1] - my_goal[1]))
                # More rovers than frontiers - fall through to greedy best.
        return max(clusters, key=lambda c: self._score(c, robot))

    def _frontier_step(self, clusters, robot):
        best = self._select_frontier(clusters, robot)
        best_score = self._score(best, robot)

        if self.navigating:
            self._check_progress(robot)
            if not self.navigating:
                return
            if self.goal_kind != GOAL_FRONTIER:
                return  # let sweep/verify goals finish on their own
            if self.goal_pos and math.hypot(
                    best['goal'][0] - self.goal_pos[0],
                    best['goal'][1] - self.goal_pos[1]) < self.replan_distance:
                return
            elapsed = self._now_sec() - (self.goal_sent_time or 0.0)
            current = next(
                (c for c in clusters if self.goal_pos and math.hypot(
                    c['goal'][0] - self.goal_pos[0],
                    c['goal'][1] - self.goal_pos[1]) < self.replan_distance),
                None)
            current_score = self._score(current, robot) if current else None
            if elapsed < self.goal_commit_time:
                return
            if current_score is not None:
                needed = current_score + \
                    (self.switch_margin - 1.0) * max(abs(current_score), 1.0)
                if best_score < needed:
                    return

        marker_bonus = self._marker_bonus(best)
        if marker_bonus > 0.0:
            self.stats['marker_priority_goals'] += 1
            self.get_logger().info(
                f'MARKER-PRIORITY FRONTIER ({best["goal"][0]:.2f}, '
                f'{best["goal"][1]:.2f}): confirmed ArUco borders unknown '
                f'space; exploration bonus={marker_bonus:.1f}')
        self._dispatch(best['goal'], best_score, robot, GOAL_FRONTIER)

    def _sweep_step(self, robot):
        """Camera-coverage sweep. Returns True if sweeping is in progress."""
        if self.navigating or self.validation_in_flight:
            self._check_progress(robot)
            return True
        # A sweep goal can "succeed" (Nav2 reached the viewpoint) without the
        # target wall ever entering the frustum (SLAM shifted the wall, yaw
        # within tolerance but unlucky, ...). Without this strike the same
        # viewpoint wins the next cycle too and the sweep livelocks.
        if self.last_sweep_target is not None:
            if not self.coverage.is_observed(self.map_msg,
                                             self.last_sweep_target):
                self._add_blacklist(self.last_sweep_target)
                self.get_logger().warn(
                    f'Sweep goal done but target '
                    f'({self.last_sweep_target[0]:.2f}, '
                    f'{self.last_sweep_target[1]:.2f}) still unobserved - '
                    f'striking it')
            self.last_sweep_target = None
        observed, total, frac = self.coverage.stats(self.map_msg)
        self._coverage_frac = frac
        self._coverage_counts = (observed, total)
        if frac >= self.coverage_target:
            return False
        clusters = self.coverage.unobserved_clusters(
            self.map_msg, self.sweep_min_cluster,
            self.sweep_max_cluster)
        clusters = [c for c in clusters
                    if not self._is_blocked(c['target'], self.blacklist)
                    and not self._is_blocked(c['target'], self.skip_list)]
        # Coordinated item search: leave walls a peer is en route to sweep
        # (its active claim) alone - shared coverage will mark them observed.
        if self.share_claims and self.peer_claims:
            clusters = [
                c for c in clusters
                if all(math.hypot(c['target'][0] - px, c['target'][1] - py)
                       > self.claim_radius
                       for px, py in self.peer_claims.values())]
        if not clusters:
            return False
        if self.state != STATE_SWEEPING:
            self.get_logger().info(
                f'Frontiers done - SWEEPING for camera coverage '
                f'({frac:.0%} of walls observed)')
            self.state = STATE_SWEEPING
        candidates = []
        for cluster in clusters:
            target = cluster['target']
            viewpoints = self.coverage.viewpoint_candidates(
                self.map_msg, target)
            if not viewpoints:
                self._add_blacklist(target)
                continue
            for vp in viewpoints:
                face_yaw = math.atan2(
                    target[1] - vp[1], target[0] - vp[0])
                gain = self.coverage.estimate_gain(
                    self.map_msg, vp[0], vp[1], face_yaw)
                if gain <= 0:
                    continue
                metrics = sweep_candidate_metrics(
                    robot, self._last_yaw, vp, target, gain,
                    self.sweep_heading_weight)
                candidates.append(
                    (metrics['utility'], cluster, vp, metrics))
        if not candidates:
            return False
        _, best, vp, metrics = max(candidates, key=lambda c: c[0])
        target = best['target']
        self.get_logger().info(
            f'Sweep selection: gain={metrics["gain"]} '
            f'distance={metrics["distance"]:.1f}m '
            f'turn={metrics["heading_cost"]:.2f}rad '
            f'utility={metrics["utility"]:.2f}')
        self._dispatch(vp, metrics['utility'], robot, GOAL_SWEEP, face=target)
        return True

    def _handle_verification(self, robot):
        """Drive to a standoff viewpoint for unconfirmed item candidates."""
        if self.verify_target is not None:
            item = self.items.get(self.verify_target)
            if item is None or item['confirmed']:
                self.verify_target = None
            else:
                self._check_progress(robot)
                return self.navigating  # still en route
        while self.pending_verify:
            mid = self.pending_verify[0]
            item = self.items.get(mid)
            if item is None or item['confirmed'] \
                    or item['attempts'] >= self.verify_max_attempts:
                self.pending_verify.popleft()
                continue
            if self.navigating or self.validation_in_flight:
                return False  # finish current goal first; verify next cycle
            vp = self.coverage.find_viewpoint(
                self.map_msg, item['pos'], robot, min_dist=0.5)
            if vp is None:
                item['attempts'] += 1
                self.pending_verify.rotate(-1)
                return False
            item['attempts'] += 1
            self.verify_target = mid
            self.stats['verify_goals'] += 1
            self.get_logger().info(
                f'Verifying item id={mid}: viewpoint '
                f'({vp[0]:.2f}, {vp[1]:.2f})')
            self._dispatch(vp, 0.0, robot, GOAL_VERIFY, face=item['pos'])
            return True
        return False

    def _dispatch(self, xy, score, robot, kind, face=None):
        if self.validation_in_flight:
            return
        if self.validate_goals and self.path_client.server_is_ready():
            self._validate_goal(xy, score, robot, kind, face)
        else:
            self._send_goal(xy, score, robot, kind, face)

    # ----------------------------------------------------- goal validation

    def _validate_goal(self, xy, score, robot, kind, face):
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(xy, robot, face)
        goal.use_start = False
        self.validation_in_flight = True
        fut = self.path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._validate_response_cb(f, xy, score, robot,
                                                 kind, face))

    def _validate_response_cb(self, future, xy, score, robot, kind, face):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.validation_in_flight = False
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._validate_result_cb(f, xy, score, robot,
                                               kind, face))

    def _validate_result_cb(self, future, xy, score, robot, kind, face):
        self.validation_in_flight = False
        result = future.result()
        ok = (result is not None
              and result.status == STATUS_SUCCEEDED
              and len(result.result.path.poses) > 0)
        if not ok:
            self.stats['validations_rejected'] += 1
            self.skips_since_dispatch += 1
            # A sweep cluster is represented by its wall target while Nav2
            # validates the standoff viewpoint. Skip the target, otherwise
            # the same unreachable viewpoint is immediately selected again.
            rejected = face if kind == GOAL_SWEEP and face is not None else xy
            self._add_skip(rejected)
            self.get_logger().info(
                f'No path to {kind} goal ({xy[0]:.2f}, {xy[1]:.2f}), skipping',
                throttle_duration_sec=20.0)
            if kind == GOAL_VERIFY and self.verify_target is not None:
                self.verify_target = None
            return
        self._send_goal(xy, score, robot, kind, face)

    # ------------------------------------------------------------ nav goals

    def _check_progress(self, robot):
        if self.goal_pos is None or not self.navigating:
            return
        d = math.hypot(self.goal_pos[0] - robot[0],
                       self.goal_pos[1] - robot[1])
        now = self.get_clock().now()
        if self.best_dist_to_goal is None or d < self.best_dist_to_goal - 0.1:
            self.best_dist_to_goal = d
            self.last_progress_time = now
            return
        elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        if elapsed <= self.progress_timeout:
            return
        if not self.cleared_costmaps_for_goal:
            self.get_logger().warn(
                f'No progress for {elapsed:.0f}s - clearing costmaps and '
                f'retrying goal ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})')
            self._clear_costmaps()
            self.cleared_costmaps_for_goal = True
            self.best_dist_to_goal = None
            self.last_progress_time = now
            return
        self.get_logger().warn(
            f'Still no progress after costmap clear, blacklisting goal '
            f'({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})')
        self._add_blacklist(self.goal_pos)
        if self.goal_kind == GOAL_VERIFY:
            self.verify_target = None
        self._cancel_goal()

    def _clear_costmaps(self):
        self.stats['costmap_clears'] += 1
        for client in (self.clear_local_client, self.clear_global_client):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _make_pose(self, xy, robot, face=None):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        if face is not None:
            yaw = math.atan2(face[1] - xy[1], face[0] - xy[0])
        else:
            yaw = math.atan2(xy[1] - robot[1], xy[0] - robot[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_goal(self, xy, score, robot, kind=GOAL_FRONTIER, face=None):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose server not ready',
                                   throttle_duration_sec=10.0)
            return
        # Count and arm the post-visit observation check only after planner
        # validation succeeds. Previously a rejected viewpoint was treated
        # as a completed visit and its wall target was blacklisted, allowing
        # the sweep to terminate at very low coverage.
        if kind == GOAL_SWEEP:
            self.stats['sweep_goals'] += 1
            self.last_sweep_target = face
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(xy, robot, face)
        self.goal_pos = xy
        self.goal_kind = kind
        self.goal_score = score
        self.goal_sent_time = self._now_sec()
        self.navigating = True
        self.best_dist_to_goal = None
        self.last_progress_time = self.get_clock().now()
        self.cleared_costmaps_for_goal = False
        self.skips_since_dispatch = 0
        self.stats['goals_sent'] += 1
        self._publish_claim(xy)
        self.get_logger().info(
            f'New {kind} goal: ({xy[0]:.2f}, {xy[1]:.2f})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, xy=xy: self._goal_response_cb(f, xy))

    def _goal_response_cb(self, future, xy):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            if xy == self.goal_pos:
                self.navigating = False
            return
        if xy == self.goal_pos or self.state == STATE_RETURNING:
            self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, xy=xy: self._result_cb(f, xy))

    def _result_cb(self, future, xy):
        if self.state == STATE_RETURNING:
            self.navigating = False
            self.goal_handle = None
            self.get_logger().info('Returned to initial pose.')
            self._save_map()
            return
        if xy != self.goal_pos:
            return  # stale result of a goal we preempted or cancelled
        self.navigating = False
        self.goal_handle = None
        kind = self.goal_kind
        status = future.result().status
        if status == STATUS_SUCCEEDED:
            self.stats['goals_succeeded'] += 1
            if kind == GOAL_FRONTIER:
                self.last_done_frontier = xy
        else:
            self.stats['goals_failed'] += 1
            self._add_blacklist(xy)
            self.get_logger().warn(
                f'{kind} goal failed (status {status}), blacklisted '
                f'({xy[0]:.2f}, {xy[1]:.2f})')
        if kind == GOAL_VERIFY:
            # Arrived (or failed): give detections a moment; if still
            # unconfirmed the queue rotates and we may try once more.
            self.verify_target = None
        self.goal_pos = None

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.navigating = False
        self.goal_pos = None
        self.goal_handle = None
        self._publish_claim(None)

    # ------------------------------------------------------------- watchdog

    def _watchdog(self):
        if self.state not in (STATE_EXPLORING, STATE_SWEEPING):
            return
        if self.map_msg is None:
            return
        now = self._now_sec()
        # Long-horizon wedge check, evaluated even while a goal is active:
        # paths can validate and Nav2 can stay "working" while the robot is
        # physically pinned (e.g. against out-of-bounds mesh through a
        # non-physical wall gap). Costmap clears and replans cannot fix
        # physics - only motion can, so escalate to the blind escape.
        # Only with active work though: a rover that is stationary because
        # every remaining frontier is long-banned is *done*, and blind
        # escapes there just churn the map and delay termination.
        if self.escape_ticks == 0 \
                and (self.navigating or self.validation_in_flight):
            window = [(t, x, y) for t, x, y in self.pos_history
                      if now - t < 120.0]
            if len(window) >= 10:
                moved = max(math.hypot(x - window[0][1], y - window[0][2])
                            for _, x, y in window)
                if moved < 0.3:
                    self.stats['watchdog_recoveries'] += 1
                    self.stats['escapes'] += 1
                    self.escape_count += 1
                    self.escape_ticks = 20  # 4 s at the 0.2 s escape timer
                    self.get_logger().warn(
                        f'Watchdog: <0.3 m movement in 120 s despite active '
                        f'work - blind '
                        f'{"reverse" if self.escape_count % 2 else "rotate"} '
                        f'escape')
                    self._cancel_goal()
                    self._clear_costmaps()
                    return
        if self.navigating or self.validation_in_flight:
            return
        if self.idle_cycles > 0:
            return
        recent = [(t, x, y) for t, x, y in self.pos_history
                  if now - t < self.watchdog_period]
        if len(recent) >= 2:
            moved = math.hypot(recent[-1][1] - recent[0][1],
                               recent[-1][2] - recent[0][2])
            if moved > 0.15:
                return
        self.stats['watchdog_recoveries'] += 1
        if self.skips_since_dispatch >= self.escape_after_skips \
                and self.escape_ticks == 0:
            # Every reachable goal is planner-rejected and we are not moving:
            # the robot is almost certainly wedged in lethal cost. Costmap
            # clears cannot fix physics - move blindly, then replan.
            self.stats['escapes'] += 1
            self.escape_count += 1
            self.escape_ticks = 20  # 4 s at the 0.2 s escape timer
            self.get_logger().warn(
                f'Watchdog: all goals planner-rejected '
                f'({self.skips_since_dispatch} skips) - blind '
                f'{"reverse" if self.escape_count % 2 else "rotate"} escape')
            return
        self.get_logger().warn(
            'Watchdog: work exists but no goal active and robot '
            'stationary - clearing costmaps')
        self._clear_costmaps()
        # Only EXPLORING gets the blacklist reset: losing map area to a stale
        # blacklist is worse than retrying. In SWEEPING the blacklist is the
        # give-up path that lets the sweep terminate - resetting it endlessly
        # resurrects unviewable wall targets.
        if self.state == STATE_EXPLORING \
                and self.stats['watchdog_recoveries'] % 3 == 0:
            self.get_logger().warn('Watchdog: resetting blacklist')
            # Spare long-banned entries (>= max strikes): those are
            # confirmed-unreachable ghosts, and reviving them re-starves
            # everything the reset is meant to unblock.
            self.blacklist = [b for b in self.blacklist
                              if b['strikes'] >= self.blacklist_max_strikes]
            for s in self.skip_list:
                s['expires'] = min(s['expires'], self._now_sec())

    def _escape_tick(self):
        if self.escape_ticks <= 0:
            return
        self.escape_ticks -= 1
        twist = Twist()
        if self.escape_ticks == 0:
            self.cmd_vel_pub.publish(twist)  # stop
            self._clear_costmaps()
            self.skips_since_dispatch = 0
            # Make skipped goals retryable right away, but keep their strike
            # memory: if escapes cycle without freeing the robot, the
            # rejection strikes must still accumulate into long bans or the
            # explorer can never conclude "this area is unreachable, give up".
            now = self._now_sec()
            for s in self.skip_list:
                s['expires'] = min(s['expires'], now)
            return
        # Cycle reverse / rotate / forward: a robot that nosed into a pocket
        # may only come free the way it went in (or after re-orienting), so
        # a reverse-only ladder can grind forever against the same mesh.
        phase = self.escape_count % 3
        if phase == 1:
            twist.linear.x = -0.2
        elif phase == 2:
            twist.angular.z = 0.9
        else:
            twist.linear.x = 0.2
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------ shutdown

    def _finish_exploration(self):
        # Stop both Nav2 and any direct recovery command before saving. The
        # multi-rover mapping configs set return_to_init=false so completion
        # means stop in place, exactly when no novel shared frontier remains.
        self._cancel_goal()
        self.cmd_vel_pub.publish(Twist())
        observed, total = self._coverage_counts
        frac = self._coverage_frac
        confirmed = [i for i, it in self.items.items() if it['confirmed']]
        self.get_logger().info(
            f'Exploration + sweep complete: camera coverage {frac:.0%} '
            f'({observed}/{total} wall cells), items found: '
            f'{sorted(confirmed)} ({len(confirmed)} confirmed, '
            f'{len(self.items)} seen); completion basis: '
            f'{self._completion_basis}')
        self._publish_claim(None)
        self._publish_items()
        if self.return_to_init and self.init_pose is not None:
            robot = self._robot_xy()
            self.state = STATE_RETURNING
            self.get_logger().info('Returning to initial pose...')
            if not self.nav_client.server_is_ready() or robot is None:
                self._save_map()
                return
            goal = NavigateToPose.Goal()
            goal.pose = self._make_pose(self.init_pose, robot)
            self.navigating = True
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                lambda f: self._goal_response_cb(f, self.init_pose))
        else:
            self._save_map()

    def _save_map(self):
        self.state = STATE_SAVING
        path = self.map_save_path
        if self.save_map_client.service_is_ready():
            req = SaveMap.Request()
            req.name.data = path
            self.save_map_client.call_async(req).add_done_callback(
                lambda f: self.get_logger().info(f'Map saved to {path}'))
        else:
            self.get_logger().warn('slam_toolbox save_map service unavailable')
        if self.serialize_client.service_is_ready():
            req = SerializePoseGraph.Request()
            req.filename = path
            self.serialize_client.call_async(req)
        self.state = STATE_DONE
        self._publish_status(0)
        self.get_logger().info('Exploration finished.')

    # -------------------------------------------------------- visualization

    def _publish_claim(self, xy):
        marker = Marker()
        # Publish claims in the common 'map' frame so peers on different map
        # frames (own-frame navigation) can compare them consistently.
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.robot_name
        marker.id = 0
        if xy is None:
            marker.action = Marker.DELETE
        else:
            shared = self._to_common(xy)
            if shared is None:
                return          # nothing to say until we share a frame
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.position.x = shared[0]
            marker.pose.position.y = shared[1]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.a = 0.8
        self.claim_pub.publish(marker)

    def _publish_frontier_markers(self, clusters):
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        for i, c in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'{self.robot_name}/frontiers'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = c['goal'][0]
            m.pose.position.y = c['goal'][1]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 0.05
            m.color.g = 1.0
            m.color.a = 0.7
            arr.markers.append(m)
        self.frontier_pub.publish(arr)

    def _publish_items(self):
        payload = {
            'robot': self.robot_name,
            'items': [
                {'id': mid, 'x': round(it['pos'][0], 3),
                 'y': round(it['pos'][1], 3), 'views': it['views'],
                 'confirmed': it['confirmed']}
                for mid, it in sorted(self.items.items())
            ],
        }
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self.items_pub.publish(msg)
        self._publish_item_claims()

        arr = MarkerArray()
        for mid, it in self.items.items():
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'{self.robot_name}/items'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = it['pos'][0]
            m.pose.position.y = it['pos'][1]
            m.pose.position.z = it.get('z', 0.3)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            if it['confirmed']:
                m.color.g = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 1.0
            m.color.a = 0.9
            arr.markers.append(m)
        self.items_marker_pub.publish(arr)

    def _coverage_m2(self):
        if self.map_msg is None:
            return 0.0
        grid = np.asarray(self.map_msg.data, dtype=np.int8)
        res = self.map_msg.info.resolution
        return float((grid >= 0).sum()) * res * res

    def _publish_status(self, n_frontiers):
        confirmed = sum(1 for it in self.items.values() if it['confirmed'])
        payload = {
            'robot': self.robot_name,
            'sim_time': round(self._now_sec(), 1),
            'state': self.state,
            'frontiers': n_frontiers,
            'coverage_m2': round(self._coverage_m2(), 1),
            'camera_coverage': round(self._coverage_frac, 3),
            'items_confirmed': confirmed,
            'items_seen': len(self.items),
            'aruco_landmarks': len(self.landmarks),
            'marker_priority_frontiers': self._marker_priority_frontier_count,
            'blacklist': len(self.blacklist),
            'navigating': self.navigating,
            'goal': list(self.goal_pos) if self.goal_pos else None,
            'goal_kind': self.goal_kind if self.goal_pos else None,
            # This timestamp is deliberately captured on the first successful
            # peer TF lookup, not inferred later from an alignment lock.  It is
            # the exact cycle on which this explorer can first use the other
            # rover's position to allocate travel goals.
            'peer_position_tracking': self._peer_tracking_started_at is not None,
            'peer_tracking_started_at': self._peer_tracking_started_at,
            'peer_positions': {
                name: [round(pos[0], 3), round(pos[1], 3)]
                for name, pos in self._peer_positions.items()
            },
            'shared_map_available': self._shared_map_available,
            'shared_alignment_locked': self._shared_alignment_locked,
            'shared_mask_active': self._shared_mask_active,
            'shared_masked_cells': self._shared_masked_cells,
            'completion_basis': self._completion_basis,
            'waiting_for_shared_completion': self._waiting_for_shared_completion,
            **{k: (round(v, 1) if isinstance(v, float) else v)
               for k, v in self.stats.items()},
        }
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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
