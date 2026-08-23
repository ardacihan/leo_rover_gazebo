#!/usr/bin/env python3
"""Score an ArUco run against the world's ground-truth marker poses.

    python3 scripts/score_aruco.py reports/night/<run>/aruco_registry.json \
        src/leo_rover_exploration/config/mock_markers_office_world.yaml

Reports, per marker: detected or missed, and the map-frame position error.
Also flags **false positives** -- an id the world does not contain, or a
detected id further than `--gross-error` from where it really is. A false
positive matters more than a miss here: a mis-registered marker sends the
rover to the wrong place, a missed one only costs another pass.

Position error mixes two things that are worth separating in the head:
detector error (corner noise, pose ambiguity) and SLAM error (the map frame
itself has drifted). With ~0.3 m of SLAM ATE, a 0.3 m marker error is SLAM,
not the detector. `--slam-ate` subtracts nothing but prints the comparison.
"""

import argparse
import json
import math
import sys

import yaml



def calibrate(samples_csv, truth, assumed_length):
    """Estimate the true marker side from the radial component of the error.

    A wrong `marker_length` scales every estimated range by exactly
    L_true / L_assumed, and moves the reported position along the camera's view
    ray by that factor. The camera position is not in the CSV, but it does not
    need to be: the reported marker sits at range `r` from the camera along the
    ray, the real one at `s * r`, so

        |truth - reported| = |s - 1| * r

    which uses only quantities the CSV and the ground truth already have.
    Errors from SLAM drift are *not* radial, so they inflate the spread rather
    than the median -- hence the median, and hence the spread being reported
    next to it.
    """
    import csv
    import statistics

    ratios = []
    with open(samples_csv) as fh:
        for row in csv.DictReader(fh):
            mid = int(row['id'])
            if mid not in truth:
                continue
            rng = float(row['range_m'])
            if rng <= 0.2:
                continue
            gt = truth[mid]
            err = math.dist((float(row['map_x']), float(row['map_y'])),
                            (gt['x'], gt['y']))
            ratios.append(err / rng)

    print()
    if len(ratios) < 10:
        print(f'marker-length check: only {len(ratios)} usable samples, skipping')
        return
    ratios.sort()
    med = statistics.median(ratios)
    lo, hi = ratios[len(ratios) // 10], ratios[-len(ratios) // 10]
    print(f'marker-length check ({len(ratios)} samples)')
    print(f'  radial error / range : median {med:.3f}  (10-90% {lo:.3f}-{hi:.3f})')
    # Reported positions land short of truth in the observed failure mode, so
    # the true marker is the larger one: s = 1 + median.
    print(f'  implied scale factor : {1.0 + med:.3f}')
    if assumed_length:
        print(f'  detector was told    : {assumed_length:.4f} m')
        print(f'  implied true length  : {assumed_length * (1.0 + med):.4f} m')
    if med < 0.05:
        print('  -> consistent with the configured length; residual is drift')
    else:
        print('  -> the configured marker_length looks wrong by '
              f'{med * 100:.0f}%; positions land short along the view ray')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('registry_json')
    ap.add_argument('truth_yaml')
    ap.add_argument('--gross-error', type=float, default=1.5,
                    help='m; beyond this a detection counts as a false positive')
    ap.add_argument('--slam-ate', type=float, default=None,
                    help='m; the run SLAM ATE, printed for context')
    ap.add_argument('--json-out', default=None)
    ap.add_argument('--samples', default=None,
                    help='per-detection CSV from the detector; enables the '
                         'marker-length check below')
    ap.add_argument('--marker-length', type=float, default=None,
                    help='the marker_length the detector was run with, so the '
                         'implied true length can be reported')
    args = ap.parse_args()

    with open(args.registry_json) as fh:
        reg = json.load(fh)
    with open(args.truth_yaml) as fh:
        truth = {int(m['id']): m for m in yaml.safe_load(fh)['markers']}

    detected = {int(m['id']): m for m in reg.get('markers', [])}

    rows, errors, false_pos = [], [], []
    for mid in sorted(truth):
        gt = truth[mid]
        if mid not in detected:
            rows.append((mid, 'MISSED', None, None))
            continue
        d = detected[mid]
        err = math.hypot(d['x'] - gt['x'], d['y'] - gt['y'])
        gross = err > args.gross_error
        if gross:
            false_pos.append((mid, err))
        else:
            errors.append(err)
        rows.append((mid, 'GROSS' if gross else 'ok', err, d))

    for mid in sorted(detected):
        if mid not in truth:
            false_pos.append((mid, float('nan')))
            rows.append((mid, 'PHANTOM-ID', None, detected[mid]))

    n_true = len(truth)
    n_ok = sum(1 for r in rows if r[1] == 'ok')
    print(f'ground-truth markers : {n_true}')
    print(f'detected correctly   : {n_ok}/{n_true}')
    print(f'missed               : {sum(1 for r in rows if r[1] == "MISSED")}')
    print(f'false positives      : {len(false_pos)}')
    print(f'frames processed     : {reg.get("frames")} '
          f'({reg.get("frames_with_detections")} contained a marker)')
    if reg.get('pending'):
        print(f'seen but unconfirmed : {reg["pending"]}')
    if errors:
        errors_sorted = sorted(errors)
        mid_i = len(errors_sorted) // 2
        print(f'position error       : mean {sum(errors)/len(errors):.3f} m, '
              f'median {errors_sorted[mid_i]:.3f} m, max {max(errors):.3f} m')
    if args.slam_ate is not None:
        print(f'(run SLAM ATE RMSE   : {args.slam_ate:.3f} m -- marker error at '
              'or below this is dominated by map drift, not the detector)')

    print()
    print(f'{"id":>3}  {"status":<11} {"err_m":>7}   detected (x, y)      truth (x, y)')
    for mid, status, err, d in rows:
        gt = truth.get(mid)
        det_s = f'({d["x"]:6.2f},{d["y"]:6.2f})' if d else '        --        '
        gt_s = f'({gt["x"]:6.2f},{gt["y"]:6.2f})' if gt else '     (no such id) '
        err_s = f'{err:7.3f}' if err is not None else '      -'
        print(f'{mid:>3}  {status:<11} {err_s}   {det_s}   {gt_s}')

    if args.samples:
        calibrate(args.samples, truth, args.marker_length)

    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({
                'n_truth': n_true,
                'n_detected_correct': n_ok,
                'n_missed': sum(1 for r in rows if r[1] == 'MISSED'),
                'n_false_positive': len(false_pos),
                'mean_error_m': (sum(errors) / len(errors)) if errors else None,
                'max_error_m': max(errors) if errors else None,
                'frames': reg.get('frames'),
                'frames_with_detections': reg.get('frames_with_detections'),
            }, fh, indent=2)

    return 0 if not false_pos else 1


if __name__ == '__main__':
    sys.exit(main())
