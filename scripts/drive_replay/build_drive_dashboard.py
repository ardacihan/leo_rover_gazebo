#!/usr/bin/env python3
"""Assemble the drive-review dashboard for one bag.

    python3 build_drive_dashboard.py <run_dir> [--title NAME] [--embed] [--compact]

<run_dir> is a reports/drive_.../<bag> directory holding default/ (from
extract_bag.py), optionally logic/ (tuned replay render, extract_replay.py)
and optionally logic_baseline/ (frozen-profile replay render). When both
logic variants exist the page grows a Tuned / Baseline / A/B switch that
swaps the stack panels, the event log, and the command chart between the
two replays. Writes <run_dir>/dashboard.html.

--embed inlines every file as a data URI (for the published artifact; use
--compact so it reads the re-encoded compact/ media and writes
dashboard_compact.html instead).
"""
import argparse
import base64
import json
from pathlib import Path

DEFAULT_PANELS = [
    ('color', 'default/color.mp4', 'Camera', 'both',
     'RealSense colour stream'),
    ('depth', 'default/depth.mp4', 'Depth', 'both',
     'colormapped 0–5 m; black = no return'),
    ('lidar', 'default/lidar.mp4', 'Lidar on raw odometry', 'default',
     'scan hits accumulated in the odom frame — drift stays visible'),
]

STACK_PANELS = [
    ('map', 'map.mp4', 'SLAM map',
     'slam_toolbox live map · plan in green · goal ★ · frontiers yellow'),
    ('gcost', 'global_costmap.mp4', 'Global costmap',
     'inflation orange–red · plan and goal overlaid'),
    ('lcost', 'local_costmap.mp4', 'Local costmap',
     '4 m rolling window · footprint green · lethal red'),
]

# Order = display order. 'logic' always holds the render of the CURRENT
# config/real; older tunings keep their own directories.
VARIANTS = [('robust', 'logic'),
            ('lidar-only', 'logic_lidar'),
            ('low-obstacle', 'logic_lowobs'),
            ('baseline', 'logic_baseline')]


def build(run_dir, title, embed, compact=False):
    run = Path(run_dir)
    data = json.loads((run / 'default' / 'data.json').read_text())
    meta = data['meta']

    def src(rel):
        p = run / ('compact/' + rel if compact else rel)
        if not p.exists():
            return None
        if embed:
            b = base64.b64encode(p.read_bytes()).decode()
            return f'data:video/mp4;base64,{b}'
        return (('compact/' + rel) if compact else rel).replace('\\', '/')

    panels = []
    for pid, rel, name, mode, note in DEFAULT_PANELS:
        s = src(rel)
        if s is not None:
            panels.append({'id': pid, 'src': s, 'name': name, 'mode': mode,
                           'note': note})

    logic_variants = {}
    for variant, subdir in VARIANTS:
        lp = run / subdir / 'logic.json'
        if not lp.exists():
            continue
        variant_panels = []
        for pid, fname, name, note in STACK_PANELS:
            s = src(f'{subdir}/{fname}')
            if s is not None:
                variant_panels.append({'id': f'{pid}_{variant}', 'src': s,
                                       'name': name, 'mode': 'logic',
                                       'variant': variant, 'note': note})
        if not variant_panels:
            continue  # e.g. compact build without this variant's media
        panels.extend(variant_panels)
        logic = json.loads(lp.read_text())
        logic_variants[variant] = {
            'goals': logic.get('goals', []),
            'events': logic.get('events', []),
            'shadow': logic.get('shadow', []),
            'cmdReplay': logic.get('cmd', []),
        }

    order = [v for v, _ in VARIANTS if v in logic_variants]
    payload = {
        'title': title,
        'duration': meta['duration_s'],
        'fps': meta['fps'],
        'distance': meta.get('distance_m'),
        'panels': panels,
        'traj': data.get('traj', []),
        'cmd': data.get('cmd', []),
        'battery': data.get('battery', []),
        'logicVariants': logic_variants,
        'variantOrder': order,
        'hasLogic': bool(logic_variants),
        'hasBaseline': len(order) > 1,
    }

    html = TEMPLATE.replace('__PAYLOAD__', json.dumps(payload)) \
                   .replace('__TITLE__', title)
    out = run / ('dashboard_compact.html' if compact else 'dashboard.html')
    out.write_text(html, encoding='utf-8')
    print(f'wrote {out} ({out.stat().st_size/1e6:.1f} MB, '
          f'{len(panels)} panels, variants={sorted(logic_variants)})')


TEMPLATE = r'''<title>__TITLE__</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root{
    --ground:#171512; --panel:#211e1a; --panel2:#2a2620; --line:#3a362f;
    --text:#e8e4dc; --muted:#9a937f; --accent:#f59b50;
    --ok:#7bc47f; --warn:#e8c452; --crit:#e86b5a; --plan:#3cdc3c;
    --baseline:#e8c452;
    --mono:'IBM Plex Mono',Consolas,monospace;
    --disp:'Chakra Petch','Segoe UI',system-ui,sans-serif;
  }
  *{box-sizing:border-box;margin:0}
  body{background:var(--ground);color:var(--text);font-family:var(--disp);
       padding:0 0 40px}
  header{display:flex;flex-wrap:wrap;align-items:center;gap:10px 22px;
         padding:14px 22px 10px;border-bottom:1px solid var(--line)}
  header h1{font-size:19px;font-weight:700;letter-spacing:.02em}
  header .stat{font-family:var(--mono);font-size:12.5px;color:var(--muted)}
  header .stat b{color:var(--text);font-weight:500}
  .seg{display:flex;border:1px solid var(--line);border-radius:6px;
       overflow:hidden}
  .seg button{background:none;border:0;color:var(--muted);cursor:pointer;
    font:600 12px var(--disp);letter-spacing:.06em;text-transform:uppercase;
    padding:7px 13px}
  .seg button.on{background:var(--accent);color:#171512}
  .seg.ab button.on{background:var(--plan);color:#171512}
  .seg button:focus-visible{outline:2px solid var(--accent);outline-offset:-2px}
  .seg .lbl{align-self:center;font:600 10px var(--disp);color:var(--muted);
            padding:0 8px;letter-spacing:.08em}
  .ctrls{margin-left:auto;display:flex;gap:10px;flex-wrap:wrap}

  .transport{display:flex;align-items:center;gap:14px;padding:12px 22px;
             position:sticky;top:0;background:var(--ground);z-index:5;
             border-bottom:1px solid var(--line)}
  .transport button{width:38px;height:38px;border-radius:50%;cursor:pointer;
    border:1px solid var(--line);background:var(--panel);color:var(--text);
    font-size:15px}
  .transport button:focus-visible{outline:2px solid var(--accent)}
  #clock{font-family:var(--mono);font-size:14px;min-width:110px}
  #clock span{color:var(--muted)}
  .rail{position:relative;flex:1;height:38px}
  .rail input{position:absolute;inset:0;width:100%;margin:0;
              -webkit-appearance:none;background:none;cursor:pointer}
  .rail input::-webkit-slider-runnable-track{height:6px;margin-top:16px;
    background:var(--panel2);border-radius:3px}
  .rail input::-webkit-slider-thumb{-webkit-appearance:none;width:14px;
    height:14px;border-radius:50%;background:var(--accent);margin-top:-4px}
  .rail canvas{position:absolute;inset:0;width:100%;height:100%;
               pointer-events:none}
  select{background:var(--panel);color:var(--text);border:1px solid var(--line);
         border-radius:5px;font:500 12px var(--mono);padding:6px}

  main{display:grid;gap:14px;padding:16px 22px;
       grid-template-columns:repeat(auto-fit,minmax(340px,1fr))}
  .pane{background:var(--panel);border:1px solid var(--line);border-radius:8px;
        overflow:hidden;display:flex;flex-direction:column}
  .pane h2{font-size:12px;font-weight:600;letter-spacing:.07em;
    text-transform:uppercase;padding:9px 12px 2px;color:var(--accent);
    display:flex;gap:8px;align-items:baseline}
  .pane .note{font-size:11.5px;color:var(--muted);padding:0 12px 8px}
  .pane video{width:100%;display:block;background:#000}
  .pane .chip{font-size:10px;border-radius:4px;padding:1px 6px;
              letter-spacing:.06em}
  .pane[data-variant="robust"] h2{color:var(--plan)}
  .pane[data-variant="robust"] .chip{background:var(--plan);color:#171512}
  .pane[data-variant="low-obstacle"] h2{color:#6fb8e8}
  .pane[data-variant="low-obstacle"] .chip{background:#6fb8e8;color:#171512}
  .pane[data-variant="lidar-only"] h2{color:#5fd8c8}
  .pane[data-variant="lidar-only"] .chip{background:#5fd8c8;color:#171512}
  .pane[data-variant="baseline"] h2{color:var(--baseline)}
  .pane[data-variant="baseline"] .chip{background:var(--baseline);color:#171512}

  .charts{padding:6px 22px;display:grid;gap:14px;
          grid-template-columns:repeat(auto-fit,minmax(420px,1fr))}
  .chart{background:var(--panel);border:1px solid var(--line);border-radius:8px;
         padding:10px 12px}
  .chart h2{font-size:12px;font-weight:600;letter-spacing:.07em;
    text-transform:uppercase;color:var(--muted);margin-bottom:6px}
  .chart canvas{width:100%;height:110px;display:block}

  .log{margin:8px 22px;background:var(--panel);border:1px solid var(--line);
       border-radius:8px;max-height:260px;overflow-y:auto}
  .log h2{font-size:12px;font-weight:600;letter-spacing:.07em;padding:10px 12px 4px;
    text-transform:uppercase;color:var(--muted);position:sticky;top:0;
    background:var(--panel)}
  .log table{width:100%;border-collapse:collapse;font:12px var(--mono)}
  .log td{padding:3px 12px;border-top:1px solid var(--line);vertical-align:top}
  .log tr{cursor:pointer}
  .log tr:hover td{background:var(--panel2)}
  .log .t{color:var(--accent);white-space:nowrap}
  .log .k-goal{color:var(--plan)} .log .k-goal_fail,.log .k-no_progress{color:var(--crit)}
  .log .k-stop,.log .k-recovery{color:var(--warn)} .log .k-explore{color:var(--muted)}
  .hidden{display:none!important}
</style>
<header>
  <h1>__TITLE__</h1>
  <span class="stat" id="s-dur"></span>
  <span class="stat" id="s-dist"></span>
  <span class="stat" id="s-goals"></span>
  <div class="ctrls">
    <div class="seg ab hidden" id="abseg" role="tablist">
      <span class="lbl">params</span>
    </div>
    <div class="seg" role="tablist">
      <button id="m-default" class="on">Default</button>
      <button id="m-logic">Rover stack</button>
      <button id="m-both">Both</button>
    </div>
  </div>
</header>
<div class="transport">
  <button id="play" aria-label="play/pause">►</button>
  <div id="clock">0:00.0 <span>/ 0:00</span></div>
  <div class="rail">
    <canvas id="ticks"></canvas>
    <input id="seek" type="range" min="0" max="1000" value="0">
  </div>
  <select id="rate">
    <option value="0.5">0.5×</option>
    <option value="1" selected>1×</option>
    <option value="2">2×</option>
    <option value="4">4×</option>
  </select>
</div>
<main id="panes"></main>
<div class="charts">
  <div class="chart"><h2>speed · m/s</h2><canvas id="c-speed"></canvas></div>
  <div class="chart" id="cmdchart"><h2 id="cmdtitle">cmd_vel driven <span style="color:var(--accent)">■</span> vs safety-chain shadow <span style="color:var(--crit)">■</span> · linear m/s</h2><canvas id="c-cmd"></canvas></div>
</div>
<div class="log" id="log"><h2 id="logtitle">stack events — click to seek</h2><table><tbody id="logbody"></tbody></table></div>
<script>
const D = __PAYLOAD__;
const $ = s => document.querySelector(s);
const dur = D.duration;
const variants = D.variantOrder || [];
let variant = variants[0] || null;   // which replay the log/chart/ticks describe
let mode = D.hasLogic ? 'both' : 'default';

document.title = D.title;
$('#s-dur').innerHTML = `duration <b>${fmt(dur)}</b>`;
if (D.distance != null) $('#s-dist').innerHTML = `driven <b>${D.distance} m</b>`;

function fmt(t){const m=Math.floor(t/60),s=(t%60);return `${m}:${s<10?'0':''}${s.toFixed(1)}`}
function logicOf(v){ return D.logicVariants[v]
                          || D.logicVariants[variants[0]] || null; }

// ---- panels
const vids = [];
for (const p of D.panels) {
  const el = document.createElement('div');
  el.className = 'pane'; el.dataset.mode = p.mode;
  if (p.variant) el.dataset.variant = p.variant;
  const chip = p.variant && D.hasBaseline
    ? `<span class="chip">${p.variant}</span>` : '';
  el.innerHTML = `<h2>${p.name}${chip}</h2><div class="note">${p.note}</div>`;
  const v = document.createElement('video');
  v.muted = true; v.playsInline = true; v.preload = 'auto'; v.src = p.src;
  el.appendChild(v); $('#panes').appendChild(el); vids.push(v);
}
const master = vids[0];

// ---- mode + variant switches
const modeBtns = {default:$('#m-default'), logic:$('#m-logic'), both:$('#m-both')};
const varBtns = {};
function applyVisibility(){
  for (const [k,b] of Object.entries(modeBtns)) b.classList.toggle('on', k===mode);
  for (const [k,b] of Object.entries(varBtns)) b.classList.toggle('on', k===variant);
  document.querySelectorAll('.pane').forEach(el=>{
    const pm = el.dataset.mode, pv = el.dataset.variant;
    let show = (mode==='both' || pm==='both' || pm===mode);
    if (show && pv) show = (variant==='split' || pv===variant);
    el.classList.toggle('hidden', !show);
  });
  const noLogic = mode==='default' || !D.hasLogic;
  $('#cmdchart').classList.toggle('hidden', noLogic);
  $('#log').classList.toggle('hidden', noLogic);
}
for (const k in modeBtns) modeBtns[k].onclick = ()=>{ mode=k; applyVisibility(); };
if (D.hasBaseline){
  const seg = $('#abseg');
  seg.classList.remove('hidden');
  for (const name of [...variants, 'split']){
    const b = document.createElement('button');
    b.textContent = name === 'split' ? 'A/B' : name;
    b.onclick = ()=>{ variant = name; applyVisibility(); refreshLogicViews(); };
    varBtns[name] = b;
    seg.appendChild(b);
  }
}

// ---- transport
let playing = false;
const seek = $('#seek');
function setTime(t, hard){
  t = Math.max(0, Math.min(dur, t));
  for (const v of vids)
    if (hard || Math.abs(v.currentTime - t) > 0.35) v.currentTime = t;
  $('#clock').innerHTML = `${fmt(t)} <span>/ ${fmt(dur)}</span>`;
  seek.value = 1000 * t / dur;
  drawCharts(t);
}
$('#play').onclick = ()=>{
  playing = !playing;
  $('#play').textContent = playing ? '❚❚' : '►';
  vids.forEach(v => playing ? v.play() : v.pause());
};
master.addEventListener('timeupdate', ()=>{ if (playing) setTime(master.currentTime) });
master.addEventListener('ended', ()=>{ playing=false; $('#play').textContent='►';
  vids.forEach(v=>v.pause()); });
seek.addEventListener('input', ()=>{ setTime(dur*seek.value/1000, true) });
$('#rate').onchange = e => vids.forEach(v=>v.playbackRate=+e.target.value);
document.addEventListener('keydown', e=>{
  if (e.key===' '){ e.preventDefault(); $('#play').click(); }
  if (e.key==='ArrowRight') setTime(master.currentTime+5, true);
  if (e.key==='ArrowLeft') setTime(master.currentTime-5, true);
});

// ---- event ticks on the rail
const KIND_COLOR = {goal:'#3cdc3c', goal_fail:'#e86b5a', no_progress:'#e86b5a',
  stop:'#e8c452', recovery:'#e8c452', explore:'#9a937f', goal_ok:'#7bc47f'};
const tickCv = $('#ticks');
function drawTicks(){
  const r = tickCv.getBoundingClientRect();
  tickCv.width = r.width*devicePixelRatio; tickCv.height = r.height*devicePixelRatio;
  const g = tickCv.getContext('2d'); g.scale(devicePixelRatio, devicePixelRatio);
  const logic = logicOf(variant==='split'?variants[0]:variant);
  if (!logic) return;
  for (const ev of logic.events){
    g.fillStyle = KIND_COLOR[ev.kind]||'#666';
    g.fillRect(r.width*ev.t/dur, 4, 1.5, 8);
  }
}
addEventListener('resize', drawTicks);

// ---- charts
function series(cv, rows, cols, t){
  const r = cv.getBoundingClientRect();
  cv.width = r.width*devicePixelRatio; cv.height = r.height*devicePixelRatio;
  const g = cv.getContext('2d'); g.scale(devicePixelRatio, devicePixelRatio);
  const W = r.width, H = r.height;
  let maxv = 0.05;
  for (const s of rows) for (const p of s) maxv = Math.max(maxv, Math.abs(p[1]));
  g.strokeStyle = '#3a362f'; g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, H-14); g.lineTo(W, H-14); g.stroke();
  rows.forEach((s, i)=>{
    g.strokeStyle = cols[i]; g.lineWidth = 1.4; g.beginPath();
    for (const p of s){
      const x = W*p[0]/dur, y = H-14-(H-24)*Math.abs(p[1])/maxv;
      p===s[0] ? g.moveTo(x,y) : g.lineTo(x,y);
    }
    g.stroke();
  });
  g.fillStyle = '#f59b50';
  g.fillRect(W*t/dur, 0, 1.2, H);
  g.fillStyle = '#9a937f'; g.font = '10px IBM Plex Mono';
  g.fillText(maxv.toFixed(2), 4, 10);
}
const speedRows = [D.traj.map(p=>[p[0], p[4]])];
function drawCharts(t){
  series($('#c-speed'), speedRows, ['#7bc47f'], t);
  const logic = logicOf(variant==='split'?variants[0]:variant);
  if (D.hasLogic && logic){
    const cmdRows = [ (logic.cmdReplay.length?logic.cmdReplay:D.cmd).map(p=>[p[0],p[1]]),
                      logic.shadow.map(p=>[p[0],p[1]]) ];
    series($('#c-cmd'), cmdRows, ['#f59b50','#e86b5a'], t);
  }
}

// ---- event log (rebuilt when the variant changes)
function refreshLogicViews(){
  const shown = variant==='split' ? variants[0] : variant;
  const logic = logicOf(shown);
  const tb = $('#logbody');
  tb.innerHTML = '';
  if (logic){
    const tag = D.hasBaseline ? ` — ${shown} params` : '';
    $('#logtitle').textContent = `stack events${tag} — click to seek`;
    $('#s-goals').innerHTML =
      `nav goals <b>${logic.goals.length}</b> · events <b>${logic.events.length}</b>`;
    for (const ev of logic.events){
      const tr = document.createElement('tr');
      tr.innerHTML = `<td class="t">${fmt(ev.t)}</td>` +
        `<td class="k-${ev.kind}">${ev.kind}</td><td>${ev.node}</td>` +
        `<td>${ev.msg.replace(/</g,'&lt;')}</td>`;
      tr.onclick = ()=>setTime(ev.t, true);
      tb.appendChild(tr);
    }
  }
  drawTicks();
  drawCharts(master ? master.currentTime : 0);
}

applyVisibility();
refreshLogicViews();
setTime(0, true);
</script>
'''


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--title', default=None)
    ap.add_argument('--embed', action='store_true')
    ap.add_argument('--compact', action='store_true',
                    help='use compact/ media (and name it dashboard_compact.html)')
    a = ap.parse_args()
    build(a.run_dir, a.title or Path(a.run_dir).name, a.embed, a.compact)
