#!/usr/bin/env python3
"""Overnight experimental suite driver.

Three studies, every run through scripts/auto_multirobot_run.sh:

  A coordination: single | independent | coordinated-unknown (hybrid) |
    coordinated-known (alignment_mode fixed), on all four worlds.
  B alignment mode: tag-only | map-only (markerfree), shorter caps,
    on office/depot/small_house (hybrid baseline comes from study A).
  C landmark count: small_house with 3 / 9 / 15 flush-validated markers.

Reproducibility: spawns come from spawn_poses.py (identical every run);
odometry noise seeds are fixed by the orchestrator (leo1=1, leo2=2); every
run's exact condition, environment and exit code goes into the manifest.

The queue is interleaved by repetition round, so stopping at the deadline
leaves every cell with a balanced number of repetitions. Hard failures
(exit not in {0,3}) are retried once at the end of the queue.

Usage:  python3 scripts/run_suite.py [--deadline-hours 9.0] [--dry-run]
"""

import argparse
import json
import os
import shutil
import subprocess
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITE = os.path.join(ROOT, 'final', 'bundles')
STAMP = datetime.now().strftime('%Y-%m-%d')
LOG = os.path.join(ROOT, 'final', f'suite_{STAMP}.log')
MANIFEST = os.path.join(ROOT, 'final', f'suite_{STAMP}_manifest.json')

# world -> (parallel slots, cap_min duals, cap singles, sim_speed, extra env)
WORLD_CFG = {
    'office_world':  dict(k=3, cap=8,  speed='2.0'),
    'depot_world':   dict(k=3, cap=7,  speed='2.0'),
    'small_house':   dict(k=3, cap=10, speed='2.0'),
    'husarion_office': dict(k=2, cap=12, speed='1.5'),
}
L_IDS = ','.join(str(i) for i in range(15))

COND = {
    'single': dict(mode='single', env={}),
    'indep':  dict(mode='independent', env={'ALIGN_MODE': 'hybrid'}),
    'c2u':    dict(mode='coordinated', env={'ALIGN_MODE': 'hybrid'}),
    'c2k':    dict(mode='coordinated', env={'ALIGN_MODE': 'fixed'}),
    'tag':    dict(mode='coordinated', env={'ALIGN_MODE': 'tag'}),
    'mfree':  dict(mode='coordinated',
                   env={'ALIGN_MODE': 'markerfree', 'SKIP_ARUCO': '1'}),
}


def build_queue():
    """[(bundle, world, cond, rep, cap, extra_env)] in priority order."""
    cells = []  # (bundle, world, cond, reps, cap, extra_env)
    for w in ('office_world', 'depot_world', 'small_house', 'husarion_office'):
        cfg = WORLD_CFG[w]
        short = {'office_world': 'office', 'depot_world': 'depot',
                 'small_house': 'small', 'husarion_office': 'husarion'}[w]
        dual_reps = {'office': 5, 'depot': 5, 'small': 4, 'husarion': 2}[short]
        single_reps = {'office': 6, 'depot': 6, 'small': 4, 'husarion': 2}[short]
        for c in ('c2u', 'indep', 'c2k'):
            cells.append((f'st-coord-{short}', w, c, dual_reps, cfg['cap'], {}))
        cells.append((f'st-coord-{short}', w, 'single', single_reps,
                      cfg['cap'], {}))
    for w, reps in (('office_world', 4), ('depot_world', 3), ('small_house', 3)):
        short = {'office_world': 'office', 'depot_world': 'depot',
                 'small_house': 'small'}[w]
        for c in ('tag', 'mfree'):
            cells.append((f'st-align-{short}', w, c, reps, 6, {}))
    for variant in ('l3', 'l9', 'l15'):
        cells.append(('st-lmk-small', f'small_house_{variant}', 'c2u', 4, 10,
                      {'ALLOWED_IDS': L_IDS}))
    # Phase-3 extras appended after the core (short caps, cheap):
    extras = []
    for w, c in (('office_world', 'c2u'), ('office_world', 'indep'),
                 ('office_world', 'c2k'), ('depot_world', 'c2u'),
                 ('depot_world', 'indep'), ('depot_world', 'c2k'),
                 ('office_world', 'single'), ('depot_world', 'single')):
        short = 'office' if 'office' in w else 'depot'
        extras.append((f'st-coord-{short}', w, c, 3, 6, {}))
    for variant in ('l3', 'l9', 'l15'):
        extras.append(('st-lmk-small', f'small_house_{variant}', 'c2u', 2, 8,
                       {'ALLOWED_IDS': L_IDS}))

    extras += [
        ('st-coord-small', 'small_house', 'c2u', 2, 8, {}),
        ('st-coord-small', 'small_house', 'indep', 2, 8, {}),
        ('st-align-office', 'office_world', 'tag', 2, 6, {}),
        ('st-align-office', 'office_world', 'mfree', 2, 6, {}),
    ]

    # Batches of up to k same-cell repetitions, round-robined across cells
    # so an early stop leaves every cell with a balanced repetition count.
    def chunks(cell_list, rep_offset=None):
        rounds = []
        for bundle, w, c, reps, cap, env in cell_list:
            k = WORLD_CFG[world_key(w)]['k']
            start = (rep_offset or {}).get((bundle, w, c), 0)
            todo = list(range(start + 1, start + reps + 1))
            n = 0
            while todo:
                take, todo = todo[:k], todo[k:]
                while len(rounds) <= n:
                    rounds.append([])
                rounds[n].append([
                    dict(bundle=bundle, world=w, cond=c, rep=r, cap=cap,
                         env=env) for r in take])
                n += 1
        out = []
        for rnd in rounds:
            out.extend(rnd)
        return out

    base_rep = {(b, w, c): r for b, w, c, r, _, _ in cells}
    return chunks(cells) + chunks(extras, base_rep)


def log(msg):
    line = f"[suite {datetime.now().strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(LOG, 'a') as fh:
        fh.write(line + '\n')


def world_key(world):
    return world if world in WORLD_CFG else 'small_house'


def launch_run(task, slot):
    """Start one orchestrator run in a subprocess; returns Popen + rundir."""
    bundle_dir = os.path.join(SUITE, task['bundle'])
    name = f"run_{task['cond']}_{task['world'].replace('_world','').replace('husarion_office','husarion').replace('small_house','sh')}_r{task['rep']}"
    # bundle pages key on run* dirs; keep names run_-prefixed and unique
    rundir = os.path.join(bundle_dir, 'runs', name)
    if os.path.exists(rundir):
        subprocess.run(['docker', 'run', '--rm', '-v', f'{ROOT}:/ros2_ws',
                        'leo_rover_humble:latest', 'bash', '-c',
                        f'rm -rf /ros2_ws/{os.path.relpath(rundir, ROOT)}'],
                       capture_output=True)
    os.makedirs(rundir, exist_ok=True)
    cfg = WORLD_CFG[world_key(task['world'])]
    env = dict(os.environ)
    env.update({
        'LEO_IMAGE': 'leo_rover_humble:latest',
        'MAX_WALL_MIN': str(task['cap'] + 12),
        'SIM_SPEED': cfg['speed'],
        'CONTAINER_NAME': f'leo_sim_r{slot}',
        'ROS_DOMAIN_ID': str(30 + slot),
        'IGN_PARTITION': f'leo_r{slot}',
        'GZ_PARTITION': f'leo_r{slot}',
    })
    env.update(COND[task['cond']]['env'])
    env.update(task['env'])
    mode = COND[task['cond']]['mode']
    out = open(os.path.join(rundir, 'host.log'), 'w')
    proc = subprocess.Popen(
        ['bash', 'scripts/auto_multirobot_run.sh', mode, task['world'],
         os.path.relpath(rundir, ROOT), str(task['cap'])],
        cwd=ROOT, env=env, stdout=out, stderr=subprocess.STDOUT)
    return proc, rundir, name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deadline-hours', type=float, default=9.0)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    queue = build_queue()
    n_runs = sum(len(b) for b in queue)
    log(f'queue built: {len(queue)} batches, {n_runs} planned runs')
    if args.dry_run:
        for b in queue:
            t = b[0]
            print(t['bundle'], t['world'], t['cond'],
                  f"reps{[x['rep'] for x in b]}", f"cap{t['cap']}")
        return 0

    t0 = time.time()
    deadline = t0 + args.deadline_hours * 3600
    manifest = {'started': datetime.now().isoformat(),
                'seeds': {'leo1_odom': 1, 'leo2_odom': 2},
                'runs': []}
    results = []
    retries = []
    successes = 0
    i = 0
    small_house_fatals = 0

    while i < len(queue) or retries:
        if time.time() > deadline:
            log(f'deadline reached at {successes} successful runs; stopping')
            break
        if shutil.disk_usage(ROOT).free < 60e9:
            log('LOW DISK (<60 GB): stopping new launches')
            break
        if i < len(queue):
            batch = queue[i]; i += 1
            if small_house_fatals >= 2 and 'small_house' in batch[0]['world']:
                batch = batch[:2]   # drop to 2-way if this world is flaky
        else:
            batch = [retries.pop(0)]
        for s in range(1, len(batch) + 1):
            subprocess.run(['docker', 'rm', '-f', f'leo_sim_r{s}'],
                           capture_output=True)
        procs = []
        for slot, task in enumerate(batch, start=1):
            proc, rundir, name = launch_run(task, slot)
            log(f"launch {name} [{task['bundle']}] slot{slot} "
                f"cap{task['cap']}")
            procs.append((proc, task, rundir, name, time.time()))
            time.sleep(20)
        for proc, task, rundir, name, ts in procs:
            rc = proc.wait()
            ok = rc in (0, 3)
            entry = dict(name=name, bundle=task['bundle'], world=task['world'],
                         cond=task['cond'], rep=task['rep'], cap=task['cap'],
                         exit=rc, ok=ok, wall_min=round((time.time()-ts)/60, 1),
                         attempt=task.get('attempt', 1),
                         align=COND[task['cond']]['env'].get(
                             'ALIGN_MODE', 'n/a'))
            manifest['runs'].append(entry)
            results.append(entry)
            if ok:
                successes += 1
            else:
                if 'small_house' in task['world'] and rc == 1:
                    small_house_fatals += 1
                if task.get('attempt', 1) < 2:
                    t2 = dict(task); t2['attempt'] = 2
                    retries.append(t2)
                    log(f'{name} FAILED rc={rc}; queued retry')
                else:
                    log(f'{name} FAILED rc={rc} twice; giving up')
            json.dump(manifest, open(MANIFEST, 'w'), indent=1)
        log(f'progress: {successes} ok / {len(results)} attempted, '
            f'{(time.time()-t0)/3600:.1f} h elapsed')

    manifest['finished'] = datetime.now().isoformat()
    manifest['successes'] = successes
    json.dump(manifest, open(MANIFEST, 'w'), indent=1)
    log(f'RUNS DONE: {successes} successful / {len(results)} attempted')

    # ---- fast finalize every touched bundle + dashboards ----
    touched = sorted({r['bundle'] for r in results})
    titles = {
        'st-coord-office': 'Suite A - coordination on office_world',
        'st-coord-depot': 'Suite A - coordination on depot_world',
        'st-coord-small': 'Suite A - coordination on small_house',
        'st-coord-husarion': 'Suite A - coordination on husarion_office',
        'st-align-office': 'Suite B - alignment modes on office_world',
        'st-align-depot': 'Suite B - alignment modes on depot_world',
        'st-align-small': 'Suite B - alignment modes on small_house',
        'st-lmk-small': 'Suite C - landmark count on small_house',
    }
    worlds = {'st-lmk-small': 'small_house'}
    for b in touched:
        w = worlds.get(b) or next(
            r['world'] for r in results if r['bundle'] == b)
        if w.startswith('small_house_'):
            w = 'small_house'
        env = dict(os.environ)
        env.update({'FAST_FINALIZE': '1',
                    'BASE': os.path.join('final', 'bundles', b, 'runs'),
                    'WORLD': w, 'TITLE': titles.get(b, b),
                    'NOTE': f'{STAMP} overnight suite'})
        log(f'finalizing {b}')
        subprocess.run(['bash', 'scripts/finalize_runs.sh'], cwd=ROOT, env=env,
                       stdout=open(LOG, 'a'), stderr=subprocess.STDOUT)
    log('SUITE COMPLETE')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
