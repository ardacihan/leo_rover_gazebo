#!/usr/bin/env python3
"""Record the leo2->leo1 alignment estimate over sim time, with its error.

The Phase 1 gate asks for the recovered transform "logged with the confidence
trace over time, not a single final number" -- a final number cannot tell a
transform that converged from one that jumped to the right answer by luck on
the last update, and cannot show when the rovers first became able to
coordinate at all.

One CSV row per sample:

    t, tag_x, tag_y, tag_yaw_deg, tag_conf,
       map_x, map_y, map_yaw_deg, map_conf,
       locked, err_xy_m, err_yaw_deg, n_leo1_tags, n_leo2_tags, n_common

`err_*` compare the *accepted* (map-based, i.e. fused) estimate against the
ground-truth offset passed on the command line. Ground truth is written to the
log only; nothing here feeds it back.

Usage (inside the container):
    python3 alignment_recorder.py <out.csv> <gt_x> <gt_y> <gt_yaw> [period_sec]
"""

import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def _yaw(msg: TransformStamped) -> float:
    return 2.0 * math.atan2(msg.transform.rotation.z, msg.transform.rotation.w)


def _wrap_deg(rad: float) -> float:
    return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))


class AlignmentRecorder(Node):
    def __init__(self, path, gt, period, candidates_path=None):
        super().__init__('alignment_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.gt = gt
        self.tag = None
        self.map = None
        self.tag_conf = float('nan')
        self.map_conf = float('nan')
        self.candidate_conf = float('nan')
        self.locked = False
        self.n_tags = {'leo1': 0, 'leo2': 0}
        self.ids = {'leo1': set(), 'leo2': set()}
        self.last_debug = {}
        self.residual_m = float('nan')
        self.geometry_ok = False
        self.candidates_path = candidates_path or os.path.join(
            os.path.dirname(path), 'alignment_candidates.jsonl')

        self.f = open(path, 'w')
        self.f.write('t,tag_x,tag_y,tag_yaw_deg,tag_conf,'
                     'map_x,map_y,map_yaw_deg,map_conf,locked,'
                     'err_xy_m,err_yaw_deg,n_leo1_tags,n_leo2_tags,n_common,'
                     'residual_m,geometry_ok,candidate_conf\n')
        self.f.flush()
        self.candidates_f = open(self.candidates_path, 'w')

        self.create_subscription(
            TransformStamped, '/estimated_transform/leo2_to_leo1',
            lambda m: setattr(self, 'tag', (m.transform.translation.x,
                                            m.transform.translation.y, _yaw(m))), 10)
        self.create_subscription(
            TransformStamped, '/map_based_transform/leo2_to_leo1',
            lambda m: setattr(self, 'map', (m.transform.translation.x,
                                            m.transform.translation.y, _yaw(m))), 10)
        self.create_subscription(
            Float32, '/tag_alignment_confidence',
            lambda m: setattr(self, 'tag_conf', float(m.data)), 10)
        self.create_subscription(
            Float32, '/alignment_confidence',
            lambda m: setattr(self, 'candidate_conf', float(m.data)), 10)
        self.create_subscription(
            Float32, '/accepted_alignment_confidence',
            lambda m: setattr(self, 'map_conf', float(m.data)), 10)
        self.create_subscription(
            Bool, '/alignment_locked',
            lambda m: setattr(self, 'locked', bool(m.data)), 10)
        self.create_subscription(
            String, '/alignment_debug_json', self._alignment_debug, 10)
        self.create_subscription(
            String, '/accepted_alignment_validation',
            self._accepted_validation, 10)
        for robot in ('leo1', 'leo2'):
            self.create_subscription(
                String, f'/{robot}/apriltag_landmarks_data',
                lambda m, r=robot: self._landmarks(r, m), 10)

        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'alignment_recorder -> {path} (ground truth '
            f'{gt[0]:.2f}, {gt[1]:.2f}, {math.degrees(gt[2]):.1f} deg)')

    def _alignment_debug(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.last_debug = data
        payload = {
            't': round(self.get_clock().now().nanoseconds / 1e9, 2),
            'confidence': data.get('final_confidence'),
            'confidence_level': data.get('confidence_level'),
            'accepted': data.get('accepted'),
            'reason': data.get('reason'),
            'ambiguity_score': data.get('ambiguity_score'),
            'common_landmark_count': data.get('common_landmark_count'),
            'top_candidates': data.get('top_candidates', []),
        }
        self.candidates_f.write(json.dumps(payload, separators=(',', ':')) + '\n')
        self.candidates_f.flush()

    def _accepted_validation(self, msg):
        try:
            data = json.loads(msg.data)
            self.residual_m = float(data['residual_m'])
            self.geometry_ok = bool(data['geometry_ok'])
        except (KeyError, TypeError, ValueError):
            pass

    def _landmarks(self, robot, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        items = data.get('landmarks', data) if isinstance(data, dict) else data
        ids = set()
        if isinstance(items, dict):
            ids = {int(k) for k in items.keys() if str(k).lstrip('-').isdigit()}
        elif isinstance(items, list):
            for it in items:
                if not isinstance(it, dict):
                    continue
                # tag_based_map_aligner emits "tag_id"; accept "id" too so this
                # keeps working if the publisher is ever swapped.
                key = 'tag_id' if 'tag_id' in it else ('id' if 'id' in it else None)
                if key is not None:
                    ids.add(int(it[key]))
        self.ids[robot] = ids
        self.n_tags[robot] = len(ids)

    @staticmethod
    def _fmt(v):
        return '' if v is None else f'{v:.4f}'

    def _tick(self):
        t = self.get_clock().now().nanoseconds / 1e9
        err_xy = err_yaw = None
        accepted = self.map if self.map is not None else self.tag
        if accepted is not None:
            err_xy = math.hypot(accepted[0] - self.gt[0], accepted[1] - self.gt[1])
            err_yaw = _wrap_deg(accepted[2] - self.gt[2])
        tag = self.tag or (None, None, None)
        mp = self.map or (None, None, None)
        common = len(self.ids['leo1'] & self.ids['leo2'])
        row = ','.join([
            f'{t:.1f}',
            self._fmt(tag[0]), self._fmt(tag[1]),
            '' if tag[2] is None else f'{math.degrees(tag[2]):.3f}',
            f'{self.tag_conf:.3f}',
            self._fmt(mp[0]), self._fmt(mp[1]),
            '' if mp[2] is None else f'{math.degrees(mp[2]):.3f}',
            f'{self.map_conf:.3f}',
            '1' if self.locked else '0',
            self._fmt(err_xy),
            '' if err_yaw is None else f'{err_yaw:.3f}',
            str(self.n_tags['leo1']), str(self.n_tags['leo2']), str(common),
            self._fmt(self.residual_m),
            '1' if self.geometry_ok else '0',
            f'{self.candidate_conf:.3f}',
        ])
        self.f.write(row + '\n')
        self.f.flush()
        if accepted is not None:
            print(f'align: t={t:.0f}s err={err_xy:.2f}m/{err_yaw:.1f}deg '
                  f'conf(tag={self.tag_conf:.2f} map={self.map_conf:.2f}) '
                  f'locked={int(self.locked)} tags={self.n_tags["leo1"]}/'
                  f'{self.n_tags["leo2"]} common={common}', flush=True)
        else:
            print(f'align: t={t:.0f}s no estimate yet '
                  f'tags={self.n_tags["leo1"]}/{self.n_tags["leo2"]} '
                  f'common={common}', flush=True)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/alignment.csv'
    gt = (float(sys.argv[2]) if len(sys.argv) > 2 else 0.0,
          float(sys.argv[3]) if len(sys.argv) > 3 else 0.0,
          float(sys.argv[4]) if len(sys.argv) > 4 else 0.0)
    period = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0
    rclpy.init()
    node = AlignmentRecorder(path, gt, period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        try:
            node.f.flush()
            node.f.close()
            node.candidates_f.flush()
            node.candidates_f.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
