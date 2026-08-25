#!/usr/bin/env python3
"""Build the self-contained two-rover dashboard from a session's artifacts.

Everything the artifact shows has to be embedded, because a published page can
reach no external host. So this walks the run directories, re-encodes each
figure as a right-sized JPEG, turns the recorded merge snapshots into a
scrubbable flipbook, and writes one HTML file with all of it inline.

Run it after the runs exist:
    build_multirobot_dashboard.py reports/multirobot_2026-08-23 -o dashboard.html
"""

import argparse
import base64
import io
import json
import math
import os
import csv
import glob

import numpy as np
from PIL import Image

# ---------------------------------------------------------------- encoding


def encode(path, max_w=1180, quality=72):
    """Right-size an image and return a data URI, or None if absent."""
    if not path or not os.path.exists(path):
        return None
    img = Image.open(path)
    if img.mode in ('RGBA', 'P', 'LA'):
        bg = Image.new('RGB', img.size, (255, 255, 255))
        img = img.convert('RGBA')
        bg.paste(img, mask=img.split()[-1])
        img = bg
    else:
        img = img.convert('RGB')
    if img.width > max_w:
        h = round(img.height * max_w / img.width)
        img = img.resize((max_w, h), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, 'JPEG', quality=quality, optimize=True, progressive=True)
    return 'data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode()


# ------------------------------------------------------------- time-lapse

FREE = (247, 247, 245)
UNKNOWN = (201, 204, 209)
OCC = (20, 24, 29)


def grid_to_img(grid, info, trails, size):
    """One occupancy grid as an RGB image with world-frame trails drawn on it."""
    if grid is None or grid.size <= 1:
        return Image.new('RGB', size, (255, 255, 255))
    a = np.zeros(grid.shape + (3,), dtype=np.uint8)
    a[:] = UNKNOWN
    a[(grid >= 0) & (grid < 50)] = FREE
    a[grid >= 50] = OCC
    a = np.flipud(a)
    img = Image.fromarray(a)

    from PIL import ImageDraw
    d = ImageDraw.Draw(img)
    ox, oy, res = info
    h = img.height
    for pts, colour in trails:
        px = []
        for (x, y) in pts:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            px.append((int(round((x - ox) / res)), h - 1 - int(round((y - oy) / res))))
        if len(px) > 1:
            d.line(px, fill=colour, width=2)
        if px:
            cx, cy = px[-1]
            d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=colour,
                      outline=(255, 255, 255))

    img.thumbnail(size, Image.NEAREST)
    canvas = Image.new('RGB', size, (255, 255, 255))
    canvas.paste(img, ((size[0] - img.width) // 2, (size[1] - img.height) // 2))
    return canvas


def load_traj(path):
    """[(t, x, y)] from a traj_recorder CSV, in that rover's own map frame."""
    out = []
    if not os.path.exists(path):
        return out
    for r in csv.DictReader(open(path)):
        try:
            out.append((float(r['t']), float(r['x']), float(r['y'])))
        except (ValueError, KeyError):
            continue
    return sorted(out)


def build_timelapse(run_dir, max_frames=80, panel=(340, 300), quality=60):
    """Three-panel flipbook: leo1 | leo2 | merged, at each recorded instant."""
    snaps = sorted(glob.glob(os.path.join(run_dir, 'timelapse', 'snap*.npz')))
    if not snaps:
        return None

    # The grids are recorded every 8 s, but the rovers are logged every 2 s.
    # Driving the animation off the *trajectory* clock and holding the most
    # recent grid gives smooth motion instead of a rover that teleports once
    # per map update -- you can actually watch them drive.
    tj = {ns: load_traj(os.path.join(run_dir, f'traj_{ns}.csv'))
          for ns in ('leo1', 'leo2')}

    stamps = []
    for path in snaps:
        with np.load(path) as d:
            stamps.append(float(d['t']))
    t0, tend = stamps[0], stamps[-1]
    n = min(max_frames, max(2, len(snaps) * 2))
    times = [t0 + (tend - t0) * i / (n - 1) for i in range(n)]

    LEO1, LEO2 = (47, 143, 208), (232, 113, 10)
    frames, meta = [], []
    si = 0
    cache = None
    for t in times:
        while si + 1 < len(stamps) and stamps[si + 1] <= t:
            si += 1
            cache = None
        if cache is None:
            cache = dict(np.load(snaps[si]))
        d = cache
        tf = d['tf']
        t1 = [(x, y) for (ts, x, y) in tj['leo1'] if ts <= t and math.isfinite(x)]
        t2 = [(x, y) for (ts, x, y) in tj['leo2'] if ts <= t and math.isfinite(x)]
        if not t1 and math.isfinite(d['p1'][0]):
            t1 = [(float(d['p1'][0]), float(d['p1'][1]))]
        if not t2 and math.isfinite(d['p2'][0]):
            t2 = [(float(d['p2'][0]), float(d['p2'][1]))]
        t2_shared = []
        if math.isfinite(tf[0]):
            c, s = math.cos(tf[2]), math.sin(tf[2])
            t2_shared = [(tf[0] + c * x - s * y, tf[1] + s * x + c * y)
                         for (x, y) in t2]

        strip = Image.new('RGB', (panel[0] * 3, panel[1]), (255, 255, 255))
        strip.paste(grid_to_img(d['leo1'], d['leo1_info'], [(t1, LEO1)], panel), (0, 0))
        strip.paste(grid_to_img(d['leo2'], d['leo2_info'], [(t2, LEO2)], panel), (panel[0], 0))
        strip.paste(grid_to_img(d['shared'], d['shared_info'],
                                [(t1, LEO1), (t2_shared, LEO2)], panel), (panel[0] * 2, 0))
        buf = io.BytesIO()
        strip.save(buf, 'JPEG', quality=quality, optimize=True)
        frames.append('data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode())
        meta.append({
            't': round(t - t0),
            'locked': bool(d['locked']) and math.isfinite(float(tf[0])),
            'tf': ([round(float(tf[0]), 2), round(float(tf[1]), 2),
                    round(math.degrees(float(tf[2])), 1)]
                   if math.isfinite(float(tf[0])) else None),
        })
    return {'frames': frames, 'meta': meta}


# --------------------------------------------------- goal + rendezvous films

def _draw_pts(d, pts, info, shape, colour, r=3, ring=False):
    ox, oy, res = info
    h = shape[0]
    for (x, y) in pts:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        cx = int(round((x - ox) / res))
        cy = h - 1 - int(round((y - oy) / res))
        box = [cx - r, cy - r, cx + r, cy + r]
        if ring:
            d.ellipse(box, outline=colour, width=2)
        else:
            d.ellipse(box, fill=colour)


def _panel_with(grid, info, size, draw_fn, label, sub=''):
    """One grid rendered to `size`, with extra artwork drawn in grid pixels."""
    from PIL import ImageDraw
    if grid is None or grid.size <= 1:
        return Image.new('RGB', size, (255, 255, 255))
    a = np.zeros(grid.shape + (3,), dtype=np.uint8)
    a[:] = UNKNOWN
    a[(grid >= 0) & (grid < 50)] = FREE
    a[grid >= 50] = OCC
    img = Image.fromarray(np.flipud(a))
    draw_fn(ImageDraw.Draw(img), grid.shape)
    img.thumbnail((size[0], size[1] - 26), Image.NEAREST)
    canvas = Image.new('RGB', size, (255, 255, 255))
    canvas.paste(img, ((size[0] - img.width) // 2, 26 + (size[1] - 26 - img.height) // 2))
    d2 = ImageDraw.Draw(canvas)
    d2.text((8, 6), label, fill=(40, 40, 40))
    if sub:
        d2.text((8, 16), sub, fill=(110, 110, 110))
    return canvas


def build_goal_film(run_dir, max_frames=56, panel=(390, 350), quality=60):
    """Two panels: each rover's map, its frontier candidates, and its chosen goal.

    The point is to show *decisions*, not just motion -- small dots are the
    frontiers the explorer was considering at that instant, the ring is the one
    it committed to, and the line joins the rover to it.
    """
    snaps = sorted(glob.glob(os.path.join(run_dir, 'timelapse', 'snap*.npz')))
    if not snaps:
        return None
    # Recordings made before the recorder captured goals and tags simply do
    # not have these fields; skip the film rather than crash the build.
    with np.load(snaps[0]) as probe:
        if 'leo1_goal' not in probe:
            return None
    if len(snaps) > max_frames:
        step = len(snaps) / max_frames
        snaps = [snaps[int(i * step)] for i in range(max_frames)]
    LEO = {'leo1': (47, 143, 208), 'leo2': (232, 113, 10)}
    GOAL = (30, 150, 90)
    frames, meta = [], []
    t0 = None
    for path in snaps:
        d = np.load(path)
        t = float(d['t']); t0 = t if t0 is None else t0
        panels, note = [], []
        for ns in ('leo1', 'leo2'):
            grid, info = d[ns], d[f'{ns}_info']
            pose = d[f'{ns}_p'] if f'{ns}_p' in d else d['p1' if ns == 'leo1' else 'p2']
            goal = d[f'{ns}_goal']
            fronts = d[f'{ns}_frontiers']
            state = str(d[f'{ns}_state'])
            kind = str(d[f'{ns}_kind'])

            def artwork(dr, shape, ns=ns, pose=pose, goal=goal, fronts=fronts, info=info):
                _draw_pts(dr, [tuple(f) for f in fronts], info, shape, (150, 150, 155), r=2)
                if math.isfinite(goal[0]):
                    _draw_pts(dr, [tuple(goal)], info, shape, GOAL, r=7, ring=True)
                    if math.isfinite(pose[0]):
                        ox, oy, res = info; h = shape[0]
                        a = (int(round((pose[0] - ox) / res)), h - 1 - int(round((pose[1] - oy) / res)))
                        b = (int(round((goal[0] - ox) / res)), h - 1 - int(round((goal[1] - oy) / res)))
                        dr.line([a, b], fill=GOAL, width=1)
                _draw_pts(dr, [tuple(pose)], info, shape, LEO[ns], r=5)

            nf = int(d[f'{ns}_nfront'])
            sub = f'{state.lower()} - {nf} frontiers'
            if math.isfinite(goal[0]):
                sub += f'  goal ({goal[0]:.1f}, {goal[1]:.1f})'
            panels.append(_panel_with(grid, info, panel, artwork, ns, sub))
            note.append(f'{ns}:{state.lower()}')
        strip = Image.new('RGB', (panel[0] * 2, panel[1]), (255, 255, 255))
        strip.paste(panels[0], (0, 0)); strip.paste(panels[1], (panel[0], 0))
        buf = io.BytesIO(); strip.save(buf, 'JPEG', quality=quality, optimize=True)
        frames.append('data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode())
        meta.append({'t': round(t - t0), 'note': ' / '.join(note)})
    return {'frames': frames, 'meta': meta}


def build_rendezvous_film(run_dir, max_frames=56, panel=(600, 545), quality=62):
    """One panel in leo1's frame: which tags each rover has found, and whether
    they have found the same ones yet. This is the film of the rendezvous."""
    snaps = sorted(glob.glob(os.path.join(run_dir, 'timelapse', 'snap*.npz')))
    if not snaps:
        return None
    # Recordings made before the recorder captured goals and tags simply do
    # not have these fields; skip the film rather than crash the build.
    with np.load(snaps[0]) as probe:
        if 'leo1_goal' not in probe:
            return None
    if len(snaps) > max_frames:
        step = len(snaps) / max_frames
        snaps = [snaps[int(i * step)] for i in range(max_frames)]
    C1, C2, COMMON = (47, 143, 208), (232, 113, 10), (30, 150, 90)
    frames, meta = [], []
    t0 = None
    for path in snaps:
        d = np.load(path)
        t = float(d['t']); t0 = t if t0 is None else t0
        tf = d['tf']
        base = d['shared'] if d['shared'].size > 1 else d['leo1']
        info = d['shared_info'] if d['shared'].size > 1 else d['leo1_info']
        ids1, pos1 = list(d['leo1_tagids']), d['leo1_tagpos']
        ids2, pos2 = list(d['leo2_tagids']), d['leo2_tagpos']
        common = set(map(int, d['common']))

        def to_shared(x, y):
            if not math.isfinite(tf[0]):
                return (float('nan'), float('nan'))
            c, s = math.cos(tf[2]), math.sin(tf[2])
            return (tf[0] + c * x - s * y, tf[1] + s * x + c * y)

        def artwork(dr, shape, info=info):
            _draw_pts(dr, [tuple(p) for p in pos1], info, shape, C1, r=5)
            if math.isfinite(tf[0]):
                _draw_pts(dr, [to_shared(*p) for p in pos2], info, shape, C2, r=5)
            for mid, p in zip(ids1, pos1):
                if int(mid) in common:
                    _draw_pts(dr, [tuple(p)], info, shape, COMMON, r=9, ring=True)
            _draw_pts(dr, [tuple(d['p1'])], info, shape, C1, r=4)
            if math.isfinite(tf[0]):
                _draw_pts(dr, [to_shared(*d['p2'])], info, shape, C2, r=4)

        conf = d['conf']
        sub = (f'leo1 tags {len(ids1)}   leo2 tags {len(ids2)}   '
               f'common {len(common)}   tag conf {conf[0]:.2f}'
               if math.isfinite(conf[0]) else
               f'leo1 tags {len(ids1)}   leo2 tags {len(ids2)}   common {len(common)}')
        lab = ('shared frame established - leo2 can be placed'
               if bool(d['locked']) and math.isfinite(tf[0])
               else 'no shared frame yet - leo2 cannot be placed')
        img = _panel_with(base, info, panel, artwork, lab, sub)
        buf = io.BytesIO(); img.save(buf, 'JPEG', quality=quality, optimize=True)
        frames.append('data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode())
        meta.append({'t': round(t - t0), 'n1': len(ids1), 'n2': len(ids2),
                     'common': len(common), 'locked': bool(d['locked']) and math.isfinite(tf[0])})
    return {'frames': frames, 'meta': meta}


# ------------------------------------------------------------------ data

def read_alignment(path, truth):
    """Convergence trace: sim time, tag error, accepted error, confidences."""
    if not os.path.exists(path):
        return None
    out = []
    for r in csv.DictReader(open(path)):
        def f(k):
            v = r.get(k, '')
            try:
                return float(v)
            except (TypeError, ValueError):
                return None
        t = f('t')
        if t is None:
            continue
        row = {'t': t, 'common': int(r.get('n_common') or 0),
               'locked': r.get('locked') == '1',
               'tconf': f('tag_conf'), 'mconf': f('map_conf')}
        for src, px, py, pyaw in (('tag', 'tag_x', 'tag_y', 'tag_yaw_deg'),
                                  ('acc', 'map_x', 'map_y', 'map_yaw_deg')):
            x, y, yaw = f(px), f(py), f(pyaw)
            if x is None or truth is None:
                row[src] = None
            else:
                row[src] = {
                    'e': round(math.hypot(x - truth[0], y - truth[1]), 3),
                    'a': round(abs((yaw - truth[2] + 180) % 360 - 180), 2)}
        out.append(row)
    if not out:
        return None
    t0 = out[0]['t']
    for r in out:
        r['t'] = round(r['t'] - t0)
    step = max(1, len(out) // 160)
    return out[::step]


def coverage_series(path):
    import re
    pat = re.compile(r't=(\d+)s known=([\d.]+)m2')
    if not os.path.exists(path):
        return []
    pts = []
    for line in open(path, errors='ignore'):
        m = pat.search(line)
        if m:
            pts.append([int(m.group(1)), float(m.group(2))])
    if not pts:
        return []
    t0 = pts[0][0]
    step = max(1, len(pts) // 120)
    return [[p[0] - t0, p[1]] for p in pts][::step]


def pick_frames(d, n=3):
    """Prefer detector debug frames (markers drawn), else plain camera frames."""
    if not os.path.isdir(d):
        return []
    det = sorted(glob.glob(os.path.join(d, 'det*.png')))
    raw = sorted(glob.glob(os.path.join(d, 'raw*.png')))
    chosen = det[:n] if det else []
    if len(chosen) < n and raw:
        stride = max(1, len(raw) // (n - len(chosen)))
        chosen += raw[::stride][:n - len(chosen)]
    return chosen


# ------------------------------------------------------------------ runs
# Verdicts and headline numbers come from PROGRESS.md, which is the ledger the
# session actually wrote as it went; nothing here is recomputed or rounded a
# second time.
RUNS = [
    dict(key='p1r1', dir='phase1_husarion_coordinated', world='husarion_office',
         label='husarion · run 1', phase='Phase 1', verdict='fail',
         truth=(2.36, -11.27, 0.0), spawn1=(0, 0, 0),
         headline='leo2 wedged at its spawn',
         note='The authored leo2 spawn put the rover nose-to-a-wall. It moved '
              '1.3 m in 25 minutes and its camera saw blank plaster for 300+ '
              'frames, so it never saw a marker and nothing downstream could '
              'happen. Everything after this run follows from fixing the spawn.',
         stats=[('leo1 mapped', '107.4 m²'), ('leo2 mapped', '16.7 m²'),
                ('common tags', '0'), ('alignment', 'never')]),
    dict(key='p1r3', dir='phase1_husarion_coordinated_run3', world='husarion_office',
         label='husarion · run 3', phase='Phase 1', verdict='fail',
         truth=(9.60, -9.95, 180.0), spawn1=(0, 0, 0),
         headline='first honest test — and it failed',
         note='First run where each map was anchored on its own rover, so the '
              'transform to recover was a real 13.8 m offset. leo2\u2019s SLAM '
              'diverged: its map shattered and its landmarks were 3–9 m wrong. '
              'The Kabsch fit still reported 0.06 m residuals and 0.82 '
              'confidence while being 114° wrong.',
         stats=[('leo1 mapped', '100.4 m²'), ('leo2 mapped', '39.6 m²'),
                ('common tags', '3'), ('alignment', '3.58 m / 65°')]),
    dict(key='p1r4', dir='phase1_husarion_coordinated_run4', world='husarion_office',
         label='husarion · run 4', phase='Phase 1', verdict='pass', star=True,
         truth=(9.60, -9.95, 180.0), spawn1=(0, 0, 0),
         headline='the result — 0.12 m from tags alone',
         note='With the IMU fused through an EKF, both rovers\u2019 SLAM held. '
              'The tag alignment recovered the 13.8 m offset to 0.12 m and '
              '1.1°, and the merged map has single-pixel continuous walls, no '
              'seam and nothing outside the building.',
         stats=[('leo1 mapped', '73.8 m²'), ('leo2 mapped', '74.6 m²'),
                ('common tags', '3'), ('alignment', '0.12 m / 1.1°')]),
    dict(key='p2depot', dir='phase2_depot_coordinated_run2', world='depot_world',
         label='depot · coordinated', phase='Phase 2', verdict='pass',
         truth=(3.0, -9.0, 180.0), spawn1=(0, 4.5, 0),
         headline='clean merge, and the grid matcher caught lying',
         note='Both explorers finished on their own. The grid matcher proposed '
              'a confident 90°-flipped match; the disagreement veto refused it '
              'and published the tag estimate instead. Every wall lands where '
              'the ground-truth raster says it should.',
         stats=[('leo1 mapped', '132.7 m²'), ('leo2 mapped', '109.0 m²'),
                ('common tags', '5'), ('alignment', '0.23 m / 0.6°')]),
    dict(key='p2tl', dir='phase2_depot_coordinated_timelapse', world='depot_world',
         label='depot · second seed', phase='Phase 2', verdict='fail',
         truth=(3.0, -9.0, 180.0), spawn1=(0, 4.5, 0),
         headline='the same setup, a much worse answer',
         note='Run again on depot to record the time-lapse below, and it came '
              'out at 2.25 m and 14.8° where the first depot run managed 0.23 m '
              'and 0.6°. Same world, same code, same conditions. Showing only '
              'the good one would be cherry-picking, so both are here — this is '
              'what n=1 buys you.',
         stats=[('leo1 mapped', '130.4 m²'), ('leo2 mapped', '108.9 m²'),
                ('common tags', '4'), ('alignment', '2.25 m / 14.8°')]),
    dict(key='p2show', dir='phase2_depot_showcase', world='depot_world',
         label='depot - the filmed run', phase='Phase 2', verdict='pass',
         showcase=True,
         truth=(3.0, -9.0, 180.0), spawn1=(0, 4.5, 0),
         headline='recorded in full, and it shows the collinearity effect live',
         note='Run again on depot with everything instrumented. It sat unlocked '
              'for six minutes on two markers that were both on the x=0 wall - '
              'collinear, so the rotation was undetermined and confidence stuck '
              'at 0.35. The moment marker 4 arrived off that line, confidence '
              'jumped to 0.78 and it locked. That is the placement rule from '
              'the lab card happening in front of you.',
         stats=[('leo1 mapped', '132.7 m2'), ('leo2 mapped', '109.7 m2'),
                ('common tags', '4'), ('alignment', '0.31 m / 1.4 deg')]),
    dict(key='p4office', dir='phase4_office_fixed', world='office_world',
         label='office - after the fixes', phase='After fixes', verdict='pass',
         showcase=True,
         truth=(11.0, -10.0, 180.0), spawn1=(-7, 5, 0),
         headline='0.63 m / 0.68 deg, where the same world used to give 37 deg',
         note='Same world, same seed count, with frontier bounds and the '
              'growing blacklist in place. Both explorers finished on their '
              'own for the first time on office. The merged map is the office '
              '- corridor, three north rooms, two south rooms, partitions '
              'where they belong - against a previous run that produced two '
              'building outlines rotated 37 degrees through each other.',
         stats=[('leo1 mapped', '179.7 m2'), ('leo2 mapped', '213.4 m2'),
                ('goals wasted', '7 of 66'), ('alignment', '0.63 m / 0.68 deg')]),
    dict(key='p4depot', dir='phase4_depot_fixed', world='depot_world',
         label='depot - after the fixes', phase='After fixes', verdict='fail',
         truth=(3.0, -9.0, 180.0), spawn1=(0, 4.5, 0),
         headline='exploration fixed, alignment still a lottery',
         note='Goal waste fell from 52% to 5% and both explorers finished, but '
              'the published transform came out 1.28 m and 19 deg off - where '
              'the run immediately before it, same code, same world, reached '
              '0.19 m. The exploration fixes hold; alignment variance does not '
              'care about them.',
         stats=[('leo1 mapped', '132.1 m2'), ('leo2 mapped', '108.6 m2'),
                ('goals wasted', '2 of 42'), ('alignment', '1.28 m / 19.2 deg')]),
    dict(key='p2indep', dir='phase2_depot_independent', world='depot_world',
         label='depot · independent', phase='Phase 2', verdict='pass',
         truth=(3.0, -9.0, 180.0), spawn1=(0, 4.5, 0),
         headline='the baseline — and it ties',
         note='The uncoordinated control. 134.5 m² merged against the '
              'coordinated run\u2019s 136.5 — a 1.015× difference, which at '
              'n=1 is a tie. Its merged map is arguably the cleanest of the '
              'session despite a worse transform.',
         stats=[('leo1 mapped', '132.2 m²'), ('leo2 mapped', '108.7 m²'),
                ('common tags', '4'), ('alignment', '0.51 m / 2.8°')]),
    dict(key='p2office', dir='phase2_office_coordinated', world='office_world',
         label='office · coordinated', phase='Phase 2', verdict='fail',
         truth=(11.0, -10.0, 180.0), spawn1=(-7, 5, 0),
         headline='six common tags, still 37° out',
         note='The most common landmarks of any run and the worst result. '
              'leo1\u2019s landmark error grows with how far it has driven — '
              '0.03 m at 3 m from its spawn, 3.33 m at 19 m. Over a 24 m world '
              'that drift exceeds what six tags can correct.',
         stats=[('leo1 mapped', '189.2 m²'), ('leo2 mapped', '214.4 m²'),
                ('common tags', '6'), ('alignment', '3.05 m / 37°')]),
]

# Night 2026-08-25 — marker-free merging, distributed mergers, A/B pairs.
# Selected with --night; run dirs are relative to that night's report root.
NIGHT_RUNS = [
    dict(key='n1r1', dir='phase1_markerfree_office', world='office_world',
         label='office · marker-free · run 1', phase='Phase 1', verdict='fail',
         truth=(11.0, -10.0, 180.0), spawn1=(0, 0, 0),
         headline='64 honest abstentions, zero merges — and the blind spot',
         note='First live run of the marker-free aligner (no ArUco nodes, no '
              'cameras). It never committed a wrong transform, but leo1 '
              'mapped only two rooms while leo2 mapped everything, and the '
              'forward-only scoring capped the visibly-correct merge at 0.30. '
              'This run bought the bidirectional/triage architecture.',
         stats=[('abstentions', '64'), ('wrong commits', '0'),
                ('leo1 known', '~102 m²'), ('merge', 'never')]),
    dict(key='n1r5', dir='phase1_markerfree_office_run5', world='office_world',
         label='office · marker-free · run 5', phase='Phase 1', verdict='pass',
         star=True,
         truth=(11.0, -10.0, 180.0), spawn1=(0, 0, 0),
         headline='marker-free lock: 0.45 m / 1.3°, cameras off',
         note='34 abstentions while the maps were disjoint, then a lock at '
              't≈645 s held to the end at confidence 0.84. Both explorers '
              'self-terminated, zero failed goals; the merged map is the '
              'whole office with single walls. The Phase 2 mask activated '
              'after the lock: leo2 stopped generating frontiers in 31,619 '
              'cells the peer had covered.',
         stats=[('lock', '0.45 m / 1.3°'), ('confidence', '0.84'),
                ('abstentions first', '34'), ('failed goals', '0')]),
    dict(key='n3part', dir='phase3_partition_office', world='office_world',
         label='office · distributed · partition', phase='Phase 3',
         verdict='pass', star=True,
         truth=(11.0, -10.0, 180.0), spawn1=(0, 0, 0),
         headline='two independent merges agree to 0.28 m; laptop killed',
         note='Each rover ran its own aligner+merger on the peer’s map. '
              'Both locked independently (0.38 m and 0.17 m from truth, no '
              'markers), their estimates compose to within 0.28 m/0.6° of '
              'identity, and both per-rover merged maps were still being '
              'served 40 s after the central bridge was killed at minute 13.',
         stats=[('leo1’s lock', '0.38 m / 0.4°'),
                ('leo2’s lock', '0.17 m / 1.0°'),
                ('mutual agreement', '0.28 m / 0.6°'),
                ('after bridge kill', 'both maps live')]),
    dict(key='n2oc', dir='phase2v_office_coordinated', world='office_world',
         label='office · coordinated (Viper)', phase='Phase 2', verdict='pass',
         truth=(11.0, -10.0, 180.0), spawn1=(0, 0, 0),
         headline='lock 0.24 m / 0.1°; 19% less duplicated coverage',
         note='Run on the Viper cluster (llvmpipe, lidar-only). Locked at '
              'conf 0.94 after 37 abstentions; the shared-map mask cut '
              'duplicated coverage to 287 m² against the baseline’s '
              '355 m², with t90 a minute faster and 14 fewer goals.',
         stats=[('lock', '0.24 m / 0.1°'), ('dup coverage', '287 m²'),
                ('t90', '495 s'), ('goals/failed', '53 / 1')]),
    dict(key='n2oi', dir='phase2v_office_independent_run2',
         world='office_world',
         label='office · independent baseline (Viper)', phase='Phase 2',
         verdict='pass',
         truth=(11.0, -10.0, 180.0), spawn1=(0, 0, 0),
         headline='the uncoordinated baseline: 355 m² covered twice',
         note='Same world, same knobs, no shared map. Finishes the same area '
              'but covers 355 m² twice (vs 287 coordinated) and needs 67 '
              'goals (vs 53).',
         stats=[('dup coverage', '355 m²'), ('t90', '555 s'),
                ('goals/failed', '67 / 1'), ('merge', 'n/a (independent)')]),
    dict(key='n2dc', dir='phase2v_depot_coordinated', world='depot_world',
         label='depot · coordinated (Viper)', phase='Phase 2', verdict='pass',
         star=True,
         truth=(3.0, -9.0, 180.0), spawn1=(0, 0, 0),
         headline='the flip-prone world locks correctly: 0.18 m / 0.0°',
         note='Depot is where the old grid matcher flipped 180° on 4 of 4 '
              'runs. The marker-free aligner abstained 11 times, then locked '
              'at 0.18 m/0.0° with confidence 0.94 — no markers anywhere. '
              'Duplicated coverage 162 m² vs the baseline’s 181 m².',
         stats=[('lock', '0.18 m / 0.0°'), ('abstentions first', '11'),
                ('dup coverage', '162 m²'), ('old matcher here', '4/4 flips')]),
    dict(key='n2di', dir='phase2v_depot_independent_run2', world='depot_world',
         label='depot · independent baseline (Viper)', phase='Phase 2',
         verdict='pass',
         truth=(3.0, -9.0, 180.0), spawn1=(0, 0, 0),
         headline='baseline: 181 m² covered twice, t90 a minute faster',
         note='Honest split result: the baseline reaches 90% a minute '
              'earlier in the small depot (the mask arrives late), but '
              'covers 181 m² twice against 162 coordinated and fails twice '
              'as many goals.',
         stats=[('dup coverage', '181 m²'), ('t90', '525 s'),
                ('goals/failed', '46 / 4'), ('merge', 'n/a (independent)')]),
]

FIGURES = [
    ('merged_map.png', 'Merged map', 'The two grids fused into one, in leo1\u2019s frame.'),
    ('traj_overlay.png', 'Both paths on the merge', 'Where each rover drove, plus the true marker positions.'),
    ('leo1_map.png', 'leo1 alone', 'One rover\u2019s own map — an input to the merge.'),
    ('leo2_map.png', 'leo2 alone', 'The other input. Compare its wall quality to leo1\u2019s.'),
    ('marker_map.png', 'Marker detections', 'Each estimate joined to truth, and error against range.'),
    ('merge_comparison.png', 'What the error costs', 'The same two maps fused two ways.'),
    ('coverage.png', 'Coverage over time', 'Area known, clipped to the world bounds.'),
    ('alignment.png', 'Alignment trace', 'Error and confidence as the estimate converges.'),
]


def build_data(root, timelapse_dir, run_specs=None):
    runs = []
    for spec in (run_specs if run_specs is not None else RUNS):
        d = os.path.join(root, spec['dir'])
        if not os.path.isdir(d):
            continue
        figs = []
        for fname, title, caption in FIGURES:
            uri = encode(os.path.join(d, fname), max_w=980, quality=66)
            if uri:
                figs.append({'title': title, 'caption': caption, 'src': uri})
        cams = []
        n_cam = 8 if spec.get('showcase') else 4
        for ns in ('leo1', 'leo2'):
            for p in pick_frames(os.path.join(d, f'frames_{ns}'), n_cam):
                uri = encode(p, max_w=460, quality=68)
                if uri:
                    cams.append({'ns': ns, 'src': uri,
                                 'label': f'{ns} · t={os.path.basename(p).split("_t")[-1][:-4]}s'})
        runs.append({
            'key': spec['key'], 'label': spec['label'], 'phase': spec['phase'],
            'verdict': spec['verdict'], 'headline': spec['headline'],
            'note': spec['note'], 'stats': spec['stats'],
            'world': spec['world'], 'star': spec.get('star', False),
            'figures': figs, 'cams': cams,
            'align': read_alignment(os.path.join(d, 'alignment.csv'), spec['truth']),
            'cov': {
                'leo1': coverage_series(os.path.join(d, 'coverage_leo1.log')),
                'leo2': coverage_series(os.path.join(d, 'coverage_leo2.log')),
                'merged': coverage_series(os.path.join(d, 'coverage.log')),
            },
        })
    worlds = []
    for name, blurb in (('depot_world', '14 × 14 m. Small enough that both rovers cover it before they ever meet.'),
                        ('office_world', '24 × 16 m. A corridor with rooms off it — the long traverses that break SLAM.'),
                        ('husarion_office', 'Mesh collisions, so it cannot be rasterized; spawns were picked from a map a rover actually drove.')):
        uri = encode(os.path.join(root, 'worlds', f'{name}.png'), max_w=900, quality=70)
        if uri:
            worlds.append({'name': name, 'blurb': blurb, 'src': uri})
    films = {}
    if timelapse_dir:
        base = os.path.join(root, timelapse_dir)
        films['timelapse'] = build_timelapse(base)
        films['goals'] = build_goal_film(base)
        films['rendezvous'] = build_rendezvous_film(base)
    return {'runs': runs, 'worlds': worlds,
            'timelapse': films.get('timelapse'),
            'goals': films.get('goals'),
            'rendezvous': films.get('rendezvous')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--timelapse-dir', default='phase4_office_fixed')
    ap.add_argument('--night', action='store_true',
                    help='use the night-2026-08-25 run list')
    args = ap.parse_args()

    data = build_data(args.root, args.timelapse_dir,
                      NIGHT_RUNS if args.night else None)
    here = os.path.dirname(os.path.abspath(__file__))
    tpl = open(os.path.join(here, 'dashboard_template.html'), encoding='utf-8').read()
    html = tpl.replace('/*__DATA__*/null', json.dumps(data))
    with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(html)
    mb = os.path.getsize(args.out) / 1e6
    n_fig = sum(len(r['figures']) for r in data['runs'])
    def nf(key):
        return len(data[key]['frames']) if data.get(key) else 0
    print(f'{args.out}: {mb:.1f} MB · {len(data["runs"])} runs · {n_fig} figures '
          f'· films: merge {nf("timelapse")}, goals {nf("goals")}, '
          f'rendezvous {nf("rendezvous")} · {len(data["worlds"])} worlds')
    if mb > 15.0:
        print('  WARNING: over 15 MB, the artifact limit is 16 MB')


if __name__ == '__main__':
    main()
