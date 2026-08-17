#!/usr/bin/env python3
"""Assemble the self-contained run-review dashboard.

Everything is inlined as data URIs because the Artifact CSP blocks external
hosts. Run from the repo root:

    python3 scripts/build_dashboard.py
"""

import base64
import html
import json
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXP = os.path.join(REPO, 'reports', 'exp')
DASH = os.path.join(EXP, '_dash')
OUT = os.path.join(EXP, 'dashboard.html')

# (dir, label, stack, world, verdict, headline)
RUNS = [
    ('orig_office_world_realistic', 'Original stack', 'orig', 'office', 'warn',
     'Explores the whole building, but the map has doubled walls — the classic yaw-drift signature.'),
    ('bundle_office_world_realistic', 'Bundle, as shipped', 'bundle', 'office', 'fail',
     'Barely moves. 4.3 m driven in a full run: DWB never commands forward motion.'),
    ('bundlerpp_office_world_realistic', 'Bundle + Pure Pursuit', 'bundle', 'office', 'warn',
     'Swapping DWB for RPP unlocks motion — 55 m driven, but the VoxelLayer still wedges it.'),
    ('ekf_office_seed1', 'Final + IMU fusion', 'final', 'office', 'pass',
     'Wheel odometry fused with gyro yaw rate. Odometry error more than halved.'),
    ('voxel_ekf_office', 'VoxelLayer A/B', 'final', 'office', 'fail',
     '6,416 raytrace failures from one frozen sensor origin. The control run is directly above.'),
    ('hybridloop2_office', 'Final configuration', 'final', 'office', 'pass',
     'Zero phantom walls, independently reviewed DEPLOY-READY. The best map of the study.'),
    ('orig_depot_world_realistic', 'Original stack', 'orig', 'depot', 'warn',
     'Maps depot perfectly, but records 5 near-misses and never finishes inside the cap.'),
    ('final_depot', 'Final configuration', 'final', 'depot', 'pass',
     'Same map quality, finished the sweep, and no near-misses.'),
    ('hybrid_husarion_office_realistic', 'Husarion office', 'final', 'husarion', 'warn',
     'A furnished, mesh-walled office. No usable ground truth, so this run is visual evidence only.'),
    ('doorway_clean', 'Doorway regression', 'final', 'fixture', 'warn',
     '7 of 8 crossings of a 0.78 m door with a 0.42 m rover. Zero contacts, zero planner failures.'),
]

STACK_LABEL = {'orig': 'ORIGINAL', 'bundle': 'BUNDLE', 'final': 'FINAL'}
WORLD_LABEL = {'office': 'office_world · 24×16 m, five rooms, 24 m corridor',
               'depot': 'depot_world · 15×15 m, partitions and obstacles',
               'husarion': 'husarion_office · furnished, mesh walls — visual evidence only',
               'fixture': 'doorway fixture · 0.78 m door, 0.42 m rover'}


def b64(path, mime):
    if not os.path.isfile(path):
        return None
    with open(path, 'rb') as fh:
        return f'data:{mime};base64,' + base64.b64encode(fh.read()).decode('ascii')


def load_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return {}


def fmt(v, nd=3, dash='—'):
    if v is None:
        return dash
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def metric_chips(m, s):
    """Chips ordered so the safety-critical numbers read first."""
    out = []

    def chip(label, value, tone='', title=''):
        out.append(
            f'<div class="chip {tone}" title="{html.escape(title)}">'
            f'<span class="chip-k">{label}</span>'
            f'<span class="chip-v">{value}</span></div>')

    contacts = s.get('contact_events')
    near = s.get('near_misses')
    if contacts is not None:
        chip('contacts', fmt(contacts, 0), 'good' if contacts == 0 else 'bad',
             'Body-centre clearance at or below the 0.22 m half-width')
    if near is not None:
        chip('near-miss', fmt(near, 0), 'good' if near == 0 else 'warn',
             'Clearance at or below 0.30 m')

    ph = m.get('phantom_frac')
    if ph is not None:
        tone = 'good' if ph < 0.05 else ('warn' if ph < 0.15 else 'bad')
        chip('phantom walls', fmt(ph), tone,
             'Fraction of mapped wall cells far from any true wall')
    cov = m.get('free_area_ratio')
    if cov is not None and 0 <= cov <= 1.5:
        tone = 'good' if cov > 0.9 else ('warn' if cov > 0.6 else 'bad')
        chip('coverage', f'{cov * 100:.0f}%', tone, 'Mapped free area vs the world’s true free area')
    iou = m.get('iou@2_aligned')
    if iou is not None:
        chip('wall IoU', fmt(iou, 2), '', 'Agreement with ground-truth walls, after removing the rigid map-frame offset')
    ate = m.get('slam_ate_rmse_m')
    if ate is not None:
        chip('SLAM ATE', f'{ate:.2f} m', '', 'Absolute trajectory error, SLAM estimate vs ground truth')
    oate = m.get('odom_ate_rmse_m')
    if oate is not None:
        chip('odom ATE', f'{oate:.2f} m', '', 'The drift SLAM had to correct')
    door = s.get('doorway_passes')
    if door is not None:
        chip('narrow gaps', fmt(door, 0), '', 'Transits through a gap with under 0.55 m of clearance')
    path = s.get('path_len_m')
    if path is not None:
        chip('driven', f'{path:.0f} m', '')
    return '\n'.join(out)


def build_card(idx, spec):
    name, label, stack, world, verdict, headline = spec
    d = os.path.join(EXP, name)
    m = load_json(os.path.join(d, 'map_score.json'))
    s = load_json(os.path.join(d, 'safety_score.json'))
    if world in ('fixture', 'husarion'):
        m = {}  # empty world / unrasterisable mesh walls: map metrics are meaningless

    traj = b64(os.path.join(DASH, f'{name}_traj.png'), 'image/png')
    mp = b64(os.path.join(d, 'timelapse_final.png'), 'image/png')
    # H.264 re-encodes: the recorder's mp4v (MPEG-4 Part 2) plays in VLC but
    # no browser decodes it, so the raw files render as broken players.
    vid = b64(os.path.join(DASH, 'vid', f'{name}.mp4'), 'video/mp4')

    media = []
    if traj:
        media.append(
            f'<figure class="media"><img src="{traj}" alt="Ground truth, odometry-only and '
            f'SLAM trajectories for {html.escape(label)}" loading="lazy" '
            f'data-zoom><figcaption>Paths driven &mdash; '
            f'<b class="k-gt">ground truth</b>, <b class="k-odom">odometry only</b>, '
            f'<b class="k-slam">SLAM</b></figcaption></figure>')
    if mp:
        media.append(
            f'<figure class="media"><img src="{mp}" alt="Final occupancy grid for '
            f'{html.escape(label)}" loading="lazy" data-zoom>'
            f'<figcaption>Final map with driven trail</figcaption></figure>')
    if vid:
        media.append(
            f'<figure class="media"><video src="{vid}" controls preload="none" '
            f'playsinline muted loop></video>'
            f'<figcaption>Time-lapse of the map building</figcaption></figure>')

    return f'''<article class="card" data-world="{world}" data-stack="{stack}">
  <header class="card-head">
    <div class="card-id">
      <span class="stack stack-{stack}">{STACK_LABEL[stack]}</span>
      <h3>{html.escape(label)}</h3>
    </div>
    <span class="verdict v-{verdict}">{verdict.upper()}</span>
  </header>
  <p class="headline">{headline}</p>
  <div class="chips">{metric_chips(m, s)}</div>
  <div class="media-grid">{''.join(media)}</div>
  <p class="runid"><span>run</span> <code>{html.escape(name)}</code></p>
</article>'''


CSS = '''
:root{
  --bg:#EDF0F4; --panel:#FFFFFF; --panel-2:#F6F8FB; --line:#D3DAE4;
  --ink:#141A22; --ink-2:#4A5768; --muted:#6C7B8E;
  --accent:#D8412A; --good:#1F7A3D; --warn:#8A6100; --bad:#B3261E;
  --gt:#1E2733; --odom:#C43B32; --slam:#2C6FCF;
  --radius:10px;
  --mono:ui-monospace,"Cascadia Mono","SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
  --sans:ui-sans-serif,system-ui,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
}
@media (prefers-color-scheme:dark){
  :root:not([data-theme="light"]){
    --bg:#0E1116; --panel:#161B22; --panel-2:#1B2129; --line:#2A323D;
    --ink:#E6EAF0; --ink-2:#B4C0CE; --muted:#8B98A9;
    --accent:#FF5A3C; --good:#3FB950; --warn:#D29922; --bad:#F85149;
    --gt:#E6EAF0; --odom:#F85149; --slam:#4C9AFF;
  }
}
:root[data-theme="dark"]{
  --bg:#0E1116; --panel:#161B22; --panel-2:#1B2129; --line:#2A323D;
  --ink:#E6EAF0; --ink-2:#B4C0CE; --muted:#8B98A9;
  --accent:#FF5A3C; --good:#3FB950; --warn:#D29922; --bad:#F85149;
  --gt:#E6EAF0; --odom:#F85149; --slam:#4C9AFF;
}
*{box-sizing:border-box}
body{
  margin:0; background:var(--bg); color:var(--ink);
  font-family:var(--sans); font-size:15px; line-height:1.55;
  -webkit-font-smoothing:antialiased;
}
.wrap{max-width:1180px; margin:0 auto; padding:32px 20px 72px; display:flex; flex-direction:column; gap:34px}
code,.mono{font-family:var(--mono)}

/* ---- masthead ---- */
.masthead{display:flex; flex-direction:column; gap:14px; border-bottom:1px solid var(--line); padding-bottom:26px}
.eyebrow{
  font-family:var(--mono); font-size:11px; letter-spacing:.14em; text-transform:uppercase;
  color:var(--muted); display:flex; gap:10px; flex-wrap:wrap; align-items:center;
}
.eyebrow .dot{width:4px;height:4px;border-radius:50%;background:var(--muted);display:inline-block}
h1{
  margin:0; font-size:clamp(28px,4.4vw,44px); line-height:1.06; letter-spacing:-.022em;
  text-wrap:balance; font-weight:660;
}
.sub{margin:0; color:var(--ink-2); max-width:66ch; font-size:16.5px}

/* ---- headline stats ---- */
.stats{display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); gap:1px;
  background:var(--line); border:1px solid var(--line); border-radius:var(--radius); overflow:hidden}
.stat{background:var(--panel); padding:15px 16px; display:flex; flex-direction:column; gap:3px}
.stat b{font-family:var(--mono); font-size:24px; font-variant-numeric:tabular-nums; letter-spacing:-.02em}
.stat span{font-size:12px; color:var(--muted); font-family:var(--mono); letter-spacing:.05em; text-transform:uppercase}
.stat.is-good b{color:var(--good)}

/* ---- verdict ---- */
.verdict-band{
  border:1px solid var(--line); border-left:3px solid var(--accent);
  background:var(--panel); border-radius:var(--radius); padding:20px 22px;
  display:flex; flex-direction:column; gap:9px;
}
.verdict-band h2{margin:0; font-size:17px; letter-spacing:-.01em}
.verdict-band p{margin:0; color:var(--ink-2); max-width:78ch}

/* ---- filters ---- */
.controls{display:flex; gap:8px; flex-wrap:wrap; align-items:center}
.controls .lbl{font-family:var(--mono); font-size:11px; letter-spacing:.12em;
  text-transform:uppercase; color:var(--muted); margin-right:4px}
button.f{
  font-family:var(--mono); font-size:12px; letter-spacing:.04em; cursor:pointer;
  background:var(--panel); color:var(--ink-2); border:1px solid var(--line);
  padding:6px 13px; border-radius:999px; transition:background .15s,color .15s,border-color .15s;
}
button.f:hover{border-color:var(--muted); color:var(--ink)}
button.f[aria-pressed="true"]{background:var(--ink); color:var(--bg); border-color:var(--ink)}
button.f:focus-visible,[data-zoom]:focus-visible{outline:2px solid var(--accent); outline-offset:2px}

/* ---- world groups ---- */
.group{display:flex; flex-direction:column; gap:16px}
.group > h2{
  margin:0; font-size:13px; font-family:var(--mono); letter-spacing:.1em; text-transform:uppercase;
  color:var(--muted); display:flex; align-items:center; gap:12px;
}
.group > h2::after{content:""; flex:1; height:1px; background:var(--line)}
.cards{display:flex; flex-direction:column; gap:16px}

/* ---- ground-truth reference ---- */
.gtref{
  margin:0; display:grid; grid-template-columns:minmax(0,300px) 1fr; gap:16px; align-items:center;
  background:var(--panel-2); border:1px dashed var(--line); border-radius:var(--radius); padding:14px 16px;
}
.gtref img{width:100%; height:auto; display:block; border-radius:6px; border:1px solid var(--line);
  background:var(--panel); cursor:zoom-in}
.gtref figcaption{font-size:13.5px; color:var(--ink-2)}
.gtref figcaption b{color:var(--ink); font-family:var(--mono); font-size:12px;
  letter-spacing:.07em; text-transform:uppercase}
@media (max-width:640px){.gtref{grid-template-columns:1fr}}

/* ---- card ---- */
.card{
  background:var(--panel); border:1px solid var(--line); border-radius:var(--radius);
  padding:18px 18px 15px; display:flex; flex-direction:column; gap:13px;
}
.card[hidden]{display:none}
.card-head{display:flex; justify-content:space-between; align-items:flex-start; gap:14px; flex-wrap:wrap}
.card-id{display:flex; align-items:center; gap:11px; flex-wrap:wrap}
.card-id h3{margin:0; font-size:17.5px; letter-spacing:-.012em; font-weight:640}
.stack{
  font-family:var(--mono); font-size:10px; letter-spacing:.11em; padding:3px 8px;
  border-radius:4px; border:1px solid var(--line); color:var(--muted); white-space:nowrap;
}
.stack-final{color:var(--accent); border-color:color-mix(in srgb,var(--accent) 40%,var(--line))}
.verdict{font-family:var(--mono); font-size:10.5px; letter-spacing:.11em; padding:4px 10px; border-radius:999px}
.v-pass{background:color-mix(in srgb,var(--good) 15%,transparent); color:var(--good)}
.v-warn{background:color-mix(in srgb,var(--warn) 17%,transparent); color:var(--warn)}
.v-fail{background:color-mix(in srgb,var(--bad) 15%,transparent); color:var(--bad)}
.headline{margin:0; color:var(--ink-2); max-width:80ch}

/* ---- chips ---- */
.chips{display:flex; flex-wrap:wrap; gap:7px}
.chip{
  display:flex; align-items:baseline; gap:7px; background:var(--panel-2);
  border:1px solid var(--line); border-radius:6px; padding:5px 10px;
}
.chip-k{font-size:11px; color:var(--muted); font-family:var(--mono); letter-spacing:.03em}
.chip-v{font-family:var(--mono); font-size:13.5px; font-variant-numeric:tabular-nums; font-weight:600}
.chip.good .chip-v{color:var(--good)}
.chip.warn .chip-v{color:var(--warn)}
.chip.bad .chip-v{color:var(--bad)}

/* ---- media ---- */
.media-grid{display:grid; grid-template-columns:repeat(auto-fit,minmax(275px,1fr)); gap:13px}
.media{margin:0; display:flex; flex-direction:column; gap:6px; min-width:0}
.media img,.media video{
  width:100%; height:auto; display:block; border-radius:7px;
  border:1px solid var(--line); background:var(--panel-2);
}
.media img[data-zoom]{cursor:zoom-in}
.media figcaption{font-size:11.5px; color:var(--muted); font-family:var(--mono); letter-spacing:.02em}
.k-gt{color:var(--gt)} .k-odom{color:var(--odom)} .k-slam{color:var(--slam)}
.runid{margin:0; font-size:11px; color:var(--muted); font-family:var(--mono)}
.runid span{letter-spacing:.1em; text-transform:uppercase}
.runid code{color:var(--ink-2)}

/* ---- lightbox ---- */
dialog.zoom{
  border:none; padding:0; background:transparent; max-width:96vw; max-height:96vh;
}
dialog.zoom::backdrop{background:rgba(6,9,13,.86)}
dialog.zoom img{max-width:96vw; max-height:92vh; border-radius:8px; display:block}
dialog.zoom button{
  position:fixed; top:14px; right:16px; font-family:var(--mono); font-size:12px;
  background:var(--panel); color:var(--ink); border:1px solid var(--line);
  border-radius:6px; padding:6px 12px; cursor:pointer;
}

/* ---- notes / footer ---- */
.notes{display:grid; grid-template-columns:repeat(auto-fit,minmax(258px,1fr)); gap:13px}
.note{background:var(--panel); border:1px solid var(--line); border-radius:var(--radius); padding:16px 17px}
.note h3{margin:0 0 6px; font-size:12px; font-family:var(--mono); letter-spacing:.09em;
  text-transform:uppercase; color:var(--muted)}
.note p{margin:0; font-size:14px; color:var(--ink-2)}
.note b{color:var(--ink)}
footer{border-top:1px solid var(--line); padding-top:18px; color:var(--muted); font-size:12.5px;
  font-family:var(--mono); display:flex; flex-wrap:wrap; gap:8px 18px}
@media (prefers-reduced-motion:reduce){*{transition:none!important; animation:none!important}}
'''

JS = '''
const buttons=[...document.querySelectorAll('button.f')];
buttons.forEach(b=>b.addEventListener('click',()=>{
  buttons.forEach(o=>o.setAttribute('aria-pressed', String(o===b)));
  const f=b.dataset.filter;
  document.querySelectorAll('.card').forEach(c=>{
    c.hidden = !(f==='all' || c.dataset.stack===f || c.dataset.world===f);
  });
  document.querySelectorAll('.group').forEach(g=>{
    g.hidden = ![...g.querySelectorAll('.card')].some(c=>!c.hidden);
  });
}));
const dlg=document.getElementById('zoom'), dimg=dlg.querySelector('img');
document.querySelectorAll('img[data-zoom]').forEach(img=>{
  img.tabIndex=0;
  const open=()=>{dimg.src=img.src; dimg.alt=img.alt; dlg.showModal();};
  img.addEventListener('click',open);
  img.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();open();}});
});
dlg.addEventListener('click',e=>{if(e.target===dlg)dlg.close();});
dlg.querySelector('button').addEventListener('click',()=>dlg.close());
'''


GT_REF = {'office': 'office_world.png', 'depot': 'depot_world.png'}


def gt_reference(world):
    """Show what the world actually looks like, so a map can be judged against it."""
    fn = GT_REF.get(world)
    if not fn:
        return ''
    src = b64(os.path.join(EXP, 'worlds', fn), 'image/png')
    if not src:
        return ''
    return (f'<figure class="gtref"><img src="{src}" alt="Ground-truth floorplan of {world}" '
            f'loading="lazy" data-zoom>'
            f'<figcaption><b>Ground truth</b> &mdash; the world as built, rasterised at the lidar '
            f'plane. Every map below should look like this.</figcaption></figure>')


def main():
    groups = []
    for world, title in (('office', WORLD_LABEL['office']),
                         ('depot', WORLD_LABEL['depot']),
                         ('husarion', WORLD_LABEL['husarion']),
                         ('fixture', WORLD_LABEL['fixture'])):
        cards = [build_card(i, r) for i, r in enumerate(RUNS) if r[3] == world]
        if cards:
            groups.append(
                f'<section class="group"><h2>{title}</h2>'
                + gt_reference(world)
                + '<div class="cards">'
                + '\n'.join(cards) + '</div></section>')

    page = f'''<title>Leo Rover Run Review</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>{CSS}</style>
<div class="wrap">

<header class="masthead">
  <div class="eyebrow">
    <span>Leo Rover</span><span class="dot"></span><span>single robot</span>
    <span class="dot"></span><span>lidar + RGBD</span><span class="dot"></span>
    <span>realistic wheel-odometry drift</span><span class="dot"></span><span>2026-08-17</span>
  </div>
  <h1>What the rover actually did</h1>
  <p class="sub">Nine runs across three environments, comparing the existing stack with the
  <code>leo_nav2_exploration</code> bundle. Every run scored by the same two tools, one simulator
  instance at a time. Click any image to enlarge.</p>
</header>

<div class="stats">
  <div class="stat is-good"><b>0</b><span>contacts, all 21 runs</span></div>
  <div class="stat is-good"><b>0.000</b><span>phantom walls, best map</span></div>
  <div class="stat"><b>7/8</b><span>doorway crossings</span></div>
  <div class="stat"><b>7</b><span>bundle bugs found</span></div>
  <div class="stat"><b>97.9%</b><span>best coverage</span></div>
</div>

<div class="verdict-band">
  <h2>Deploy the bundle &mdash; with the fixes, and after a hardware bring-up</h2>
  <p>The bundle was undeployable as shipped: four defects stopped it launching or moving at all,
  three more were silent. Fixed, and with Pure Pursuit replacing DWB, <code>explore_lite</code>
  replacing its frontier explorer, and the camera on an ObstacleLayer, it matches the original
  stack on mapping and beats it decisively on safety &mdash; <b>0&ndash;1 near-misses against
  3&ndash;5</b>, and 27&ndash;32 narrow-gap transits against 3&ndash;4. What remains unproven is
  sensor reality: RealSense floor noise, the measured footprint, and CPU budget on the Orin.</p>
</div>

<div class="controls">
  <span class="lbl">Filter</span>
  <button class="f" data-filter="all" aria-pressed="true">All runs</button>
  <button class="f" data-filter="orig" aria-pressed="false">Original stack</button>
  <button class="f" data-filter="bundle" aria-pressed="false">Bundle</button>
  <button class="f" data-filter="final" aria-pressed="false">Final config</button>
  <button class="f" data-filter="office" aria-pressed="false">office_world</button>
  <button class="f" data-filter="depot" aria-pressed="false">depot_world</button>
</div>

{''.join(groups)}

<section class="notes">
  <div class="note"><h3>Reading the trajectories</h3><p>White is ground truth, blue is the SLAM
  estimate, red is odometry alone. Where red wanders outside the building and blue still hugs
  white, <b>SLAM is correcting metres of drift</b> &mdash; that gap is the whole job.</p></div>
  <div class="note"><h3>Why husarion is missing</h3><p>Its walls are <code>.dae</code> meshes the
  world rasteriser cannot see, so its ground truth is nearly empty and <b>every map metric against
  it is meaningless</b>. Those runs exist but are visual-only.</p></div>
  <div class="note"><h3>Aligned metrics</h3><p>slam_toolbox anchors the map frame on the first
  scan, so every run carries a rigid offset unrelated to quality. IoU and RMSE are reported
  <b>after removing it</b>; raw ATE is not, and is inflated by the same amount.</p></div>
  <div class="note"><h3>The corridor caveat</h3><p>Phantom walls ranged <b>0.000 to 0.217</b> across
  four identical office seeds. A 24&nbsp;m corridor exceeds the 12&nbsp;m lidar. Depot showed no such
  spread &mdash; it is a property of long hallways, not of the configuration.</p></div>
</section>

<footer>
  <span>reports/exp/FINDINGS.md</span><span>reports/exp/BUNDLE_BUGS.md</span>
  <span>reports/exp/comparison.md</span><span>21 scored runs</span>
</footer>
</div>

<dialog class="zoom" id="zoom"><button type="button">Close</button><img alt=""></dialog>
<script>{JS}</script>'''

    with open(OUT, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(page)
    mb = os.path.getsize(OUT) / 1e6
    print(f'wrote {OUT}  ({mb:.1f} MB)')
    if mb > 15:
        print('WARNING: approaching the 16 MB artifact limit')


if __name__ == '__main__':
    main()
