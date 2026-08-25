#!/usr/bin/env python3
"""Serve and record a live, read-only dashboard for the two-rover ROS graph.

The node subscribes to the same topics used by RViz and the run recorders.  It
does not publish commands, transforms, goals, or maps, so opening the dashboard
cannot influence an experiment.  A compact JSON snapshot is appended to
``telemetry.jsonl`` every two seconds and the HTML client polls the local HTTP
API for current values and occupancy-map images.

Usage (inside the ROS workspace/container):

    python3 scripts/live_multirobot_dashboard.py \
      --output reports/live_2026-08-25/interactive_office --port 8080
"""

import argparse
import copy
import json
import math
import os
import shutil
import struct
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


def _yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def _age(stamp):
    return None if not stamp else round(max(0.0, time.monotonic() - stamp), 2)


def _image_to_bgr(msg):
    """Convert common raw ROS image encodings without requiring cv_bridge."""
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        image = data.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return (cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if enc == 'rgb8' else image.copy())
    if enc in ('rgba8', 'bgra8'):
        image = data.reshape(msg.height, msg.step // 4, 4)[:, :msg.width]
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(image, code)
    if enc in ('mono8', '8uc1'):
        image = data.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported camera encoding {msg.encoding!r}')


class DashboardNode(Node):
    """Collect a deliberately small summary of the high-rate ROS graph."""

    def __init__(self, output_dir):
        super().__init__('live_multirobot_dashboard')
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.telemetry_path = self.output_dir / 'telemetry.jsonl'
        self.lock = threading.RLock()
        self.started_wall = time.time()
        self.started_mono = time.monotonic()
        self.sim_time = 0.0
        self.maps = {}
        self.camera_jpegs = {}
        self.merged_markers = []
        self.history = deque(maxlen=900)
        self.events = deque(maxlen=100)
        self._record_count = 0
        self._candidate_frame_index = len(list(
            (self.output_dir / 'candidate_maps').glob('candidate_*.jpg')))
        self._last_stage = None
        self.node_names = []
        self.alignment = {
            'locked': False, 'confidence': None, 'candidate_confidence': None,
            'tag_confidence': None,
            'transform': None, 'debug': {}, 'updated': None,
        }
        self._restore_recorded_estimation_start()
        self.robots = {
            name: {
                'pose': None, 'speed': {'linear': 0.0, 'angular': 0.0},
                'command': {'linear': 0.0, 'angular': 0.0},
                'scan': {'hz': 0.0, 'age': None, 'samples': 0},
                'camera': {'hz': 0.0, 'age': None, 'source': None,
                           'frame': None, 'width': None, 'height': None},
                'tags': 0, 'detections': 0, 'tag_markers': [],
                'frontiers': 0, 'frontier_goals': [],
                'status': 'waiting', 'goal': None,
                'map_pose': None, 'merged_pose': None,
                'mission': {},
                'map': {'known_m2': 0.0, 'free_m2': 0.0,
                        'occupied_m2': 0.0, 'updated': None},
            }
            for name in ('leo1', 'leo2')
        }
        self._restore_recorded_peer_tracking()
        self._rates = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        reliable.durability = DurabilityPolicy.VOLATILE
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor = QoSProfile(depth=2)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Bool, '/alignment_locked', self._alignment_locked, reliable)
        self.create_subscription(
            Float32, '/vetted_alignment_confidence', self._alignment_confidence,
            reliable)
        self.create_subscription(
            Float32, '/alignment_confidence', self._candidate_confidence,
            reliable)
        self.create_subscription(
            Float32, '/tag_alignment_confidence', self._tag_confidence,
            reliable)
        self.create_subscription(
            String, '/alignment_debug_json', self._alignment_debug, reliable)
        self.create_subscription(
            TransformStamped, '/vetted_transform/leo2_to_leo1',
            self._alignment_transform, reliable)

        for name in self.robots:
            self.create_subscription(
                Odometry, f'/{name}/odometry/filtered',
                lambda msg, n=name: self._odom(n, msg), sensor)
            self.create_subscription(
                Odometry, f'/{name}/wheel_odom',
                lambda msg, n=name: self._odom(n, msg), sensor)
            self.create_subscription(
                Twist, f'/{name}/cmd_vel',
                lambda msg, n=name: self._cmd(n, msg), sensor)
            self.create_subscription(
                LaserScan, f'/{name}/scan',
                lambda msg, n=name: self._rate(n, 'scan', len(msg.ranges)),
                sensor)
            # Prefer the detector's annotated image. Raw sim and RealSense
            # feeds remain fallbacks, so the same dashboard works in Gazebo
            # and on the physical rovers.
            for topic, source in (
                    (f'/{name}/aruco/debug_image', 'aruco annotated'),
                    (f'/{name}/camera/image', 'camera raw'),
                    (f'/{name}/camera/camera/color/image_raw', 'camera raw')):
                self.create_subscription(
                    Image, topic,
                    lambda msg, n=name, s=source: self._camera_image(n, s, msg),
                    sensor)
            self.create_subscription(
                MarkerArray, f'/{name}/tag_detections',
                lambda msg, n=name: self._markers(n, 'detections', msg),
                reliable)
            # tag_detections is intentionally a current-frame stream.  The
            # detector's aruco_markers output is its accumulated, map-frame
            # registry, which is what belongs on a persistent local-map view.
            self.create_subscription(
                MarkerArray, f'/{name}/aruco_markers',
                lambda msg, n=name: self._markers(n, 'tags', msg), latched)
            self.create_subscription(
                MarkerArray, f'/{name}/frontier_explorer/frontiers',
                lambda msg, n=name: self._markers(n, 'frontiers', msg),
                reliable)
            self.create_subscription(
                String, f'/{name}/frontier_explorer/status',
                lambda msg, n=name: self._status(n, msg), reliable)
            self.create_subscription(
                PoseStamped, f'/{name}/goal_pose',
                lambda msg, n=name: self._goal(n, msg), reliable)
            self.create_subscription(
                OccupancyGrid, f'/{name}/map',
                lambda msg, n=name: self._map(n, msg), reliable)

        self.create_subscription(
            OccupancyGrid, '/shared_map',
            lambda msg: self._map('shared', msg), reliable)
        self.create_subscription(
            OccupancyGrid, '/shared_map_candidate',
            lambda msg: self._map('candidate', msg), reliable)
        self.create_subscription(
            MarkerArray, '/shared/apriltag_landmarks',
            self._merged_markers, reliable)
        self.create_timer(2.0, self._record)
        self.create_timer(0.5, self._tf_update)
        self.create_timer(5.0, self._graph_update)

        template = Path(__file__).with_name('live_dashboard.html')
        if template.exists():
            shutil.copyfile(template, self.output_dir / 'dashboard.html')
        with open(self.output_dir / 'session.json', 'w', encoding='utf-8') as fh:
            json.dump({
                'started': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                'dashboard': 'dashboard.html',
                'telemetry': 'telemetry.jsonl',
            }, fh, indent=2)

    def _rate(self, robot, source, samples):
        now = time.monotonic()
        key = (robot, source)
        previous = self._rates.get(key)
        self._rates[key] = now
        with self.lock:
            data = self.robots[robot][source]
            if previous and now > previous:
                instant = 1.0 / (now - previous)
                data['hz'] = round(0.8 * data['hz'] + 0.2 * instant, 1)
            data['_stamp'] = now
            if samples is not None:
                data['samples'] = samples

    def _restore_recorded_estimation_start(self):
        """Keep the true first estimate time when a dashboard is restarted."""
        if not self.telemetry_path.is_file():
            return
        earliest = None
        try:
            with self.telemetry_path.open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                        value = record.get('alignment', {}).get(
                            'estimation_started_at')
                        if value is not None:
                            value = float(value)
                            earliest = value if earliest is None else min(
                                earliest, value)
                    except (AttributeError, TypeError, ValueError,
                            json.JSONDecodeError):
                        continue
        except OSError:
            return
        if earliest is not None:
            self.alignment['estimation_started_at'] = earliest

    def _restore_recorded_peer_tracking(self):
        """Retain each explorer's exact first peer-use time after UI restarts."""
        if not self.telemetry_path.is_file():
            return
        restored = {name: None for name in self.robots}
        positions = {name: {} for name in self.robots}
        last_status = {name: None for name in self.robots}
        last_mission = {name: {} for name in self.robots}
        last_goal = {name: None for name in self.robots}
        try:
            with self.telemetry_path.open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    for name in self.robots:
                        robot = record.get('robots', {}).get(name, {})
                        status = robot.get('status')
                        if status not in (None, 'waiting', 'unknown'):
                            last_status[name] = status
                        mission = robot.get('mission', {})
                        if mission:
                            last_mission[name].update(mission)
                        if robot.get('goal') is not None:
                            last_goal[name] = robot['goal']
                        started = mission.get('peer_tracking_started_at')
                        if started is not None:
                            started = float(started)
                            restored[name] = (
                                started if restored[name] is None else min(
                                    restored[name], started))
                        if mission.get('peer_positions'):
                            positions[name] = mission['peer_positions']
        except (AttributeError, OSError, TypeError, ValueError):
            return
        for name, started in restored.items():
            if last_status[name] is not None:
                self.robots[name]['status'] = last_status[name]
            self.robots[name]['mission'].update(last_mission[name])
            self.robots[name]['goal'] = last_goal[name]
            if started is not None:
                self.robots[name]['mission'].update({
                    'peer_position_tracking': True,
                    'peer_tracking_started_at': started,
                    'peer_positions': positions[name],
                })

    def _odom(self, robot, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        with self.lock:
            self.robots[robot]['pose'] = {
                'x': round(p.x, 3), 'y': round(p.y, 3),
                'yaw': round(_yaw(q), 1), 'frame': msg.header.frame_id,
            }
            self.robots[robot]['speed'] = {
                'linear': round(t.linear.x, 3),
                'angular': round(t.angular.z, 3),
            }

    def _cmd(self, robot, msg):
        with self.lock:
            self.robots[robot]['command'] = {
                'linear': round(msg.linear.x, 3),
                'angular': round(msg.angular.z, 3),
            }

    def _camera_image(self, robot, source, msg):
        """Keep a compact latest frame; annotated ArUco images take priority."""
        now = time.monotonic()
        with self.lock:
            current = self.robots[robot]['camera']
            annotated_fresh = (
                current.get('source') == 'aruco annotated'
                and current.get('_stamp') is not None
                and now - current['_stamp'] < 2.0)
        if source != 'aruco annotated' and annotated_fresh:
            return
        try:
            bgr = _image_to_bgr(msg)
            ok, encoded = cv2.imencode(
                '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if not ok:
                return
        except (ValueError, cv2.error) as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
            return
        self._rate(robot, 'camera', None)
        with self.lock:
            self.camera_jpegs[robot] = bytes(encoded)
            self.robots[robot]['camera'].update({
                'source': source,
                'frame': msg.header.frame_id,
                'width': int(msg.width),
                'height': int(msg.height),
                '_stamp': now,
            })

    def _markers(self, robot, field, msg):
        points = []
        seen = set()
        for marker in msg.markers:
            # Frontier streams begin with DELETEALL; tag streams may repeat an
            # id as a label. Only ADD-like position records belong on the map.
            if marker.action not in (0,):
                continue
            key = int(marker.id)
            if key in seen:
                continue
            seen.add(key)
            points.append({
                'id': key,
                'x': round(float(marker.pose.position.x), 3),
                'y': round(float(marker.pose.position.y), 3),
                'frame': marker.header.frame_id,
            })
        with self.lock:
            self.robots[robot][field] = len(points)
            if field == 'tags':
                self.robots[robot]['tag_markers'] = points
            elif field == 'frontiers':
                self.robots[robot]['frontier_goals'] = points

    def _merged_markers(self, msg):
        points = []
        seen = set()
        for marker in msg.markers:
            if marker.action != 0 or marker.ns != 'landmarks':
                continue
            mid = int(marker.id)
            if mid in seen:
                continue
            seen.add(mid)
            points.append({
                'id': mid,
                'x': round(float(marker.pose.position.x), 3),
                'y': round(float(marker.pose.position.y), 3),
                'frame': marker.header.frame_id,
            })
        with self.lock:
            self.merged_markers = points

    @staticmethod
    def _pose_from_transform(tf):
        p, q = tf.transform.translation, tf.transform.rotation
        return {
            'x': round(float(p.x), 3), 'y': round(float(p.y), 3),
            'yaw': round(_yaw(q), 1), 'frame': tf.header.frame_id,
        }

    def _tf_update(self):
        """Resolve poses in their own maps and in the accepted merged frame."""
        for robot in self.robots:
            own = merged = None
            # Simulation uses base_link; the physical rover's navigation and
            # safety configs use base_footprint. Resolve whichever the active
            # TF tree actually provides.
            for base in (f'{robot}/base_link', f'{robot}/base_footprint'):
                try:
                    own_tf = self.tf_buffer.lookup_transform(
                        f'{robot}/map', base, rclpy.time.Time(),
                        timeout=Duration(seconds=0.05))
                    own = self._pose_from_transform(own_tf)
                    break
                except Exception:
                    continue
            for base in (f'{robot}/base_link', f'{robot}/base_footprint'):
                try:
                    merged_tf = self.tf_buffer.lookup_transform(
                        'leo1/map', base, rclpy.time.Time(),
                        timeout=Duration(seconds=0.05))
                    merged = self._pose_from_transform(merged_tf)
                    break
                except Exception:
                    continue
            if merged is None and robot == 'leo1':
                merged = own
            with self.lock:
                if own is not None:
                    self.robots[robot]['map_pose'] = own
                if merged is not None:
                    self.robots[robot]['merged_pose'] = merged

    def _status(self, robot, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        with self.lock:
            if not isinstance(payload, dict):
                self.robots[robot]['status'] = msg.data
                return
            self.robots[robot]['status'] = payload.get('state', 'unknown')
            self.robots[robot]['mission'] = {
                key: payload.get(key) for key in (
                    'goals_sent', 'goals_succeeded', 'goals_failed',
                    'distance_traveled', 'camera_coverage', 'blacklist',
                    'watchdog_recoveries', 'navigating', 'goal_kind',
                    'peer_position_tracking', 'peer_tracking_started_at',
                    'peer_positions', 'shared_map_available',
                    'shared_alignment_locked',
                    'shared_mask_active', 'shared_masked_cells',
                    'peer_goals_cancelled', 'completion_basis',
                    'waiting_for_shared_completion', 'aruco_landmarks',
                    'marker_priority_frontiers', 'marker_priority_goals')
            }
            if payload.get('goal') and len(payload['goal']) >= 2:
                self.robots[robot]['goal'] = {
                    'x': round(float(payload['goal'][0]), 2),
                    'y': round(float(payload['goal'][1]), 2),
                    'frame': f'{robot}/map',
                }
            else:
                self.robots[robot]['goal'] = None

    def _goal(self, robot, msg):
        p = msg.pose.position
        with self.lock:
            self.robots[robot]['goal'] = {
                'x': round(p.x, 2), 'y': round(p.y, 2),
                'frame': msg.header.frame_id,
            }

    def _map(self, name, msg):
        values = list(msg.data)
        known = sum(v >= 0 for v in values)
        occupied = sum(v >= 50 for v in values)
        free = sum(0 <= v < 50 for v in values)
        res = float(msg.info.resolution)
        summary = {
            'known_m2': round(known * res * res, 2),
            'free_m2': round(free * res * res, 2),
            'occupied_m2': round(occupied * res * res, 2),
            'width': msg.info.width, 'height': msg.info.height,
            'resolution': res, 'frame': msg.header.frame_id,
            'origin': {
                'x': round(float(msg.info.origin.position.x), 3),
                'y': round(float(msg.info.origin.position.y), 3),
                'yaw': round(_yaw(msg.info.origin.orientation), 3),
            },
            'updated': time.monotonic(),
        }
        with self.lock:
            self.maps[name] = {
                'width': msg.info.width, 'height': msg.info.height,
                'data': values, 'summary': summary,
            }
            if name in self.robots:
                self.robots[name]['map'] = summary

    def _alignment_locked(self, msg):
        with self.lock:
            self.alignment['locked'] = bool(msg.data)
            self.alignment['updated'] = time.monotonic()

    def _alignment_confidence(self, msg):
        with self.lock:
            self.alignment['confidence'] = round(float(msg.data), 3)
            self.alignment['updated'] = time.monotonic()

    def _candidate_confidence(self, msg):
        with self.lock:
            self.alignment['candidate_confidence'] = round(float(msg.data), 3)

    def _tag_confidence(self, msg):
        with self.lock:
            self.alignment['tag_confidence'] = round(float(msg.data), 3)

    def _alignment_debug(self, msg):
        try:
            debug = json.loads(msg.data)
        except (TypeError, ValueError):
            debug = {'message': msg.data}
        with self.lock:
            self.alignment['debug'] = debug
            started = debug.get('relative_estimation_started_at')
            if started is not None:
                started = float(started)
                previous = self.alignment.get('estimation_started_at')
                self.alignment['estimation_started_at'] = (
                    started if previous is None else min(previous, started))
            self.alignment['updated'] = time.monotonic()

    def _alignment_transform(self, msg):
        t, q = msg.transform.translation, msg.transform.rotation
        with self.lock:
            self.alignment['transform'] = {
                'x': round(t.x, 3), 'y': round(t.y, 3),
                'yaw': round(_yaw(q), 2),
            }
            self.alignment['updated'] = time.monotonic()

    def _graph_update(self):
        try:
            nodes = self.get_node_names_and_namespaces()
        except Exception:
            nodes = []
        with self.lock:
            self.node_count = len(nodes)
            self.node_names = sorted(
                f"{namespace.rstrip('/')}/{name}" if namespace != '/' else f'/{name}'
                for name, namespace in nodes)

    @staticmethod
    def _has_node(node_names, suffix):
        return any(name == suffix or name.endswith(suffix) for name in node_names)

    def _execution_summary(self, robots, alignment, shared, node_names):
        sensor_ready = all(
            r['scan'].get('age') is not None and r['scan']['age'] < 3.0
            and r['camera'].get('age') is not None and r['camera']['age'] < 3.0
            for r in robots.values())
        maps_ready = all(r['map'].get('known_m2', 0.0) > 0.0
                         for r in robots.values())
        detector_nodes = {
            name: (
                self._has_node(node_names, f'/{name}/aruco_detector')
                or (robot['camera'].get('source') == 'aruco annotated'
                    and robot['camera'].get('age') is not None
                    and robot['camera']['age'] < 3.0)
            )
            for name, robot in robots.items()
        }
        aligner_running = self._has_node(node_names, '/map_based_aligner')
        merger_running = self._has_node(node_names, '/shared_map_merger')
        explorers_running = all(
            self._has_node(node_names, f'/{name}/frontier_explorer')
            for name in robots)
        estimating = bool(
            alignment.get('estimation_started_at') is not None
            or alignment.get('debug'))
        shared_live = shared.get('age') is not None and shared['age'] < 5.0
        tracking = any(
            bool(r.get('mission', {}).get('peer_position_tracking'))
            for r in robots.values())
        mission_states = [str(r.get('status', 'unknown'))
                          for r in robots.values()]
        all_done = all(state == 'done' for state in mission_states)
        finalizing = any(state in ('returning', 'saving')
                         for state in mission_states)
        waiting_global = any(bool(r.get('mission', {}).get(
            'waiting_for_shared_completion')) for r in robots.values())

        if all_done:
            current = ('MISSION_COMPLETE',
                       'No novel shared frontier remains; both rovers are stopped.')
        elif finalizing:
            current = ('FINALIZING_MISSION',
                       'Shared exploration is complete; saving mission outputs.')
        elif waiting_global:
            current = ('WAITING_FOR_SHARED_FRAME',
                       'Local work is exhausted; rover is stopped pending a vetted union check.')
        elif not sensor_ready:
            current = ('WAITING_FOR_SENSORS',
                       'Both lidar and camera feeds must be fresh.')
        elif not maps_ready:
            current = ('BUILDING_LOCAL_MAPS',
                       'SLAM is producing the two independent maps.')
        elif not all(detector_nodes.values()):
            current = ('ARUCO_PROGRAM_MISSING',
                       'At least one real aruco_detector node is not running.')
        elif not estimating:
            current = ('WAITING_FOR_USABLE_MAP_GEOMETRY',
                       'Grid alignment is armed; the local maps do not yet contain enough geometry for a candidate.')
        elif not alignment.get('locked'):
            current = ('ESTIMATING_RELATIVE_POSITION',
                       'A candidate exists but is not trusted for fusion yet.')
        elif not shared_live:
            current = ('STARTING_MAP_MERGE',
                       'The transform is locked; waiting for the merged grid.')
        elif not tracking:
            current = ('MERGING_MAPS',
                       'Merged map is live; explorers have not used a peer pose yet.')
        else:
            current = ('COORDINATED_TRAVEL',
                       'Peer positions are resolved and influence goal allocation.')

        steps = [
            ('sensor feeds', sensor_ready),
            ('local SLAM maps', maps_ready),
            ('real ArUco detectors', all(detector_nodes.values())),
            ('relative-position estimate', estimating),
            ('alignment accepted', bool(alignment.get('locked'))),
            ('merged map live', shared_live),
            ('peer tracking used for travel', tracking),
        ]
        own_total = sum(r['map'].get('known_m2', 0.0) for r in robots.values())
        shared_area = shared.get('known_m2', 0.0)
        return {
            'stage': current[0], 'detail': current[1],
            'steps': [{'name': name, 'complete': complete}
                      for name, complete in steps],
            'programs': {
                **{f'{name}_aruco_detector': running
                   for name, running in detector_nodes.items()},
                'map_aligner': aligner_running,
                'shared_map_merger': merger_running,
                'explorers': explorers_running,
            },
            'merge_progress': {
                'known_m2': shared_area,
                'candidate_known_m2': shared.get('candidate_known_m2'),
                'raw_local_sum_m2': round(own_total, 2),
                'union_bound_pct': (round(100.0 * shared_area / own_total, 1)
                                    if own_total > 0.0 else None),
            },
        }

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            robots = copy.deepcopy(self.robots)
            alignment = copy.deepcopy(self.alignment)
            shared = copy.deepcopy(
                self.maps.get('shared', {}).get('summary', {}))
            candidate = copy.deepcopy(
                self.maps.get('candidate', {}).get('summary', {}))
            node_count = getattr(self, 'node_count', 0)
            node_names = list(self.node_names)
            merged_markers = copy.deepcopy(self.merged_markers)
            events = list(self.events)
        for robot in robots.values():
            for source in ('scan', 'camera'):
                robot[source]['age'] = _age(robot[source].pop('_stamp', None))
            robot['map']['age'] = _age(robot['map'].pop('updated', None))
        shared['age'] = _age(shared.pop('updated', None))
        candidate['age'] = _age(candidate.pop('updated', None))
        shared['candidate_known_m2'] = candidate.get('known_m2')
        alignment['age'] = _age(alignment.pop('updated', None))
        execution = self._execution_summary(
            robots, alignment, shared, node_names)
        return {
            'wall_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'wall_elapsed': round(now - self.started_mono, 1),
            'sim_time': round(self.get_clock().now().nanoseconds / 1e9, 2),
            'node_count': node_count,
            'robots': robots, 'alignment': alignment, 'shared_map': shared,
            'candidate_map': candidate,
            'merged_markers': merged_markers,
            'execution': execution, 'events': events,
        }

    def _record(self):
        snap = self.snapshot()
        point = {
            't': snap['wall_elapsed'], 'sim': snap['sim_time'],
            'leo1': snap['robots']['leo1']['map']['known_m2'],
            'leo2': snap['robots']['leo2']['map']['known_m2'],
            'shared': snap['shared_map'].get('known_m2', 0.0),
            'candidate': snap['candidate_map'].get('known_m2', 0.0),
            'confidence': snap['alignment']['confidence'],
            'locked': snap['alignment']['locked'],
        }
        with self.lock:
            self.history.append(point)
            stage = snap['execution']['stage']
            if stage != self._last_stage:
                self._last_stage = stage
                event = {
                    'wall_time': snap['wall_time'],
                    'sim_time': snap['sim_time'],
                    'stage': stage,
                    'detail': snap['execution']['detail'],
                }
                self.events.append(event)
                # Make the transition unmistakable in dashboard.log as well
                # as in telemetry.jsonl.
                self.get_logger().info(
                    f"EXECUTION STAGE -> {stage} at t={snap['sim_time']:.1f}s: "
                    f"{snap['execution']['detail']}")
        with open(self.telemetry_path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(snap, separators=(',', ':')) + '\n')
        self._record_count += 1
        if self._record_count == 1 or self._record_count % 10 == 0:
            self._save_candidate_frame(snap['sim_time'])
        if self._record_count == 1 or self._record_count % 5 == 0:
            self._write_offline_dashboard(snap)

    def _write_offline_dashboard(self, snap):
        """Freeze a self-contained status/history view for post-run review."""
        template = Path(__file__).with_name('live_dashboard.html')
        if not template.exists():
            return
        payload = json.dumps({
            'status': snap,
            'history': self.history_snapshot(),
            'maps': {
                name: f'map_{name}.bmp'
                for name in ('leo1', 'leo2', 'shared', 'candidate')
                if name in self.maps
            },
            'cameras': {
                name: f'camera_{name}.jpg' for name in self.camera_jpegs
            },
        }, separators=(',', ':')).replace('</', '<\\/')
        html = template.read_text(encoding='utf-8').replace(
            '<script id="offline-data" type="application/json">null</script>',
            f'<script id="offline-data" type="application/json">{payload}</script>')
        for name in ('leo1', 'leo2', 'shared', 'candidate'):
            body = self.map_bmp(name)
            if body is not None:
                tmp = self.output_dir / f'.map_{name}.bmp.tmp'
                tmp.write_bytes(body)
                os.replace(tmp, self.output_dir / f'map_{name}.bmp')
        with self.lock:
            camera_frames = dict(self.camera_jpegs)
        for name, body in camera_frames.items():
            tmp = self.output_dir / f'.camera_{name}.jpg.tmp'
            tmp.write_bytes(body)
            os.replace(tmp, self.output_dir / f'camera_{name}.jpg')
        tmp = self.output_dir / '.dashboard.html.tmp'
        tmp.write_text(html, encoding='utf-8')
        os.replace(tmp, self.output_dir / 'dashboard.html')

    def _save_candidate_frame(self, sim_time):
        """Save a sparse visual history of unvetted merge attempts."""
        with self.lock:
            source = copy.deepcopy(self.maps.get('candidate'))
        if not source:
            return
        width, height = source['width'], source['height']
        grid = np.asarray(source['data'], dtype=np.int16).reshape(height, width)
        image = np.empty((height, width, 3), dtype=np.uint8)
        image[grid < 0] = (88, 78, 72)
        image[grid >= 50] = (27, 22, 18)
        image[(grid >= 0) & (grid < 50)] = (241, 244, 242)
        image = np.flipud(image)
        longest = max(width, height)
        if longest > 720:
            scale = 720.0 / longest
            image = cv2.resize(
                image, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_NEAREST)
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 84])
        if not ok:
            return
        directory = self.output_dir / 'candidate_maps'
        directory.mkdir(parents=True, exist_ok=True)
        name = (f'candidate_{self._candidate_frame_index:04d}_'
                f't{float(sim_time):010.1f}.jpg')
        target = directory / name
        temporary = directory / f'.{name}.tmp'
        temporary.write_bytes(bytes(encoded))
        os.replace(temporary, target)
        self._candidate_frame_index += 1

    def history_snapshot(self):
        with self.lock:
            return list(self.history)

    def map_bmp(self, name):
        """Return a browser-native 24-bit BMP without third-party imaging."""
        with self.lock:
            source = self.maps.get(name)
            if not source:
                return None
            width, height = source['width'], source['height']
            data = list(source['data'])
        row_bytes = (width * 3 + 3) & ~3
        pad = b'\0' * (row_bytes - width * 3)
        pixels = bytearray()
        # ROS occupancy data and positive-height BMP both begin at the bottom.
        for y in range(height):
            start = y * width
            for value in data[start:start + width]:
                if value < 0:
                    pixels.extend((72, 78, 88))
                elif value >= 50:
                    pixels.extend((18, 22, 27))
                else:
                    pixels.extend((242, 244, 241))
            pixels.extend(pad)
        header_size = 14 + 40
        header = struct.pack(
            '<2sIHHI', b'BM', header_size + len(pixels), 0, 0, header_size)
        dib = struct.pack(
            '<IiiHHIIiiII', 40, width, height, 1, 24, 0, len(pixels),
            2835, 2835, 0, 0)
        return header + dib + pixels

    def camera_jpeg(self, name):
        with self.lock:
            return self.camera_jpegs.get(name)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = 'LeoDashboard/1.0'

    def log_message(self, fmt, *args):
        return

    def _send(self, body, content_type, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store, max-age=0')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        node = self.server.dashboard_node
        if path in ('/', '/dashboard.html'):
            body = self.server.html_path.read_bytes()
            return self._send(body, 'text/html; charset=utf-8')
        if path == '/api/status':
            body = json.dumps(node.snapshot(), separators=(',', ':')).encode()
            return self._send(body, 'application/json')
        if path == '/api/history':
            body = json.dumps(node.history_snapshot(),
                              separators=(',', ':')).encode()
            return self._send(body, 'application/json')
        if path.startswith('/map/') and path.endswith('.bmp'):
            name = path[5:-4]
            body = node.map_bmp(name)
            if body is not None:
                return self._send(body, 'image/bmp')
        if path.startswith('/camera/') and path.endswith('.jpg'):
            name = path[8:-4]
            body = node.camera_jpeg(name)
            if body is not None:
                return self._send(body, 'image/jpeg')
        self._send(b'not found\n', 'text/plain', HTTPStatus.NOT_FOUND)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='reports/live')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--bind', default='0.0.0.0')
    # Keep standard ROS arguments (notably use_sim_time) available to
    # rclpy.init while hiding them from argparse.
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = DashboardNode(args.output)
    html_path = Path(__file__).with_name('live_dashboard.html')
    server = ThreadingHTTPServer((args.bind, args.port), DashboardHandler)
    server.dashboard_node = node
    server.html_path = html_path
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(
        f'dashboard ready on http://localhost:{args.port} '
        f'(recording to {node.output_dir})')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
