#!/usr/bin/env python3
"""Generate the collaborative-exploration report (self-contained HTML with
base64-embedded figures) into reports/collab_final/report.html.

Honest version: the system and algorithm work (rovers divide the space), but a
clean single-vs-two-robot speedup is not established by these single runs
because run-to-run variance is on the same order as the effect."""

import base64
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIGDIR = os.path.join(ROOT, 'reports', 'collab_final', 'figures')


def b64(name):
    p = os.path.join(FIGDIR, name)
    if not os.path.exists(p):
        return None
    with open(p, 'rb') as f:
        return 'data:image/png;base64,' + base64.b64encode(f.read()).decode()


def fig(name, cap):
    d = b64(name)
    if not d:
        return ''
    return (f'<figure><div class="fig-wrap"><img src="{d}" alt="{cap}"></div>'
            f'<figcaption>{cap}</figcaption></figure>')


STYLE = """
<style>
:root{
  --paper:#f5f6f7; --card:#ffffff; --ink:#161b21; --body:#39424c;
  --muted:#727a83; --line:#e2e6ea; --line-2:#eef1f3;
  --single:#4C72B0; --uncoord:#DD8452; --coord:#2f9e6b; --accent:#3a6ea5;
  --accent-soft:#e8eef5; --warn:#c9702e;
}
@media (prefers-color-scheme:dark){:root{
  --paper:#0f1319; --card:#161c24; --ink:#e9edf1; --body:#b7c0c9;
  --muted:#7f8994; --line:#232c36; --line-2:#1b2129;
  --single:#7aa0d8; --uncoord:#e8a077; --coord:#4fc08a; --accent:#6ea3d8;
  --accent-soft:#13202c; --warn:#e8a077;
}}
:root[data-theme="dark"]{
  --paper:#0f1319; --card:#161c24; --ink:#e9edf1; --body:#b7c0c9;
  --muted:#7f8994; --line:#232c36; --line-2:#1b2129;
  --single:#7aa0d8; --uncoord:#e8a077; --coord:#4fc08a; --accent:#6ea3d8;
  --accent-soft:#13202c; --warn:#e8a077;
}
:root[data-theme="light"]{
  --paper:#f5f6f7; --card:#ffffff; --ink:#161b21; --body:#39424c;
  --muted:#727a83; --line:#e2e6ea; --line-2:#eef1f3;
  --single:#4C72B0; --uncoord:#DD8452; --coord:#2f9e6b; --accent:#3a6ea5;
  --accent-soft:#e8eef5; --warn:#c9702e;
}
*{box-sizing:border-box}
body{background:var(--paper);color:var(--body);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",system-ui,sans-serif;
  line-height:1.62;margin:0;-webkit-font-smoothing:antialiased}
.wrap{max-width:860px;margin:0 auto;padding:clamp(1.5rem,4vw,4rem) 1.4rem 6rem}
.mono{font-family:ui-monospace,"SF Mono","Cascadia Code","Roboto Mono",monospace}
h1,h2,h3{color:var(--ink);text-wrap:balance;line-height:1.2;letter-spacing:-.01em}
h1{font-size:clamp(1.9rem,4.6vw,2.8rem);margin:.2em 0 .3em;font-weight:680}
h2{font-size:1.42rem;margin:2.6rem 0 .8rem;font-weight:640;
  padding-top:1.5rem;border-top:1px solid var(--line)}
h3{font-size:1.02rem;margin:1.4rem 0 .3rem;font-weight:640}
p{margin:.7rem 0}
.eyebrow{font-size:.74rem;letter-spacing:.16em;text-transform:uppercase;
  color:var(--accent);font-weight:700}
.lede{font-size:1.15rem;color:var(--body);max-width:62ch}
.meta{color:var(--muted);font-size:.85rem;margin-top:1rem;
  display:flex;gap:1.1rem;flex-wrap:wrap}
figure{margin:1.8rem 0}
.fig-wrap{background:var(--card);border:1px solid var(--line);border-radius:12px;
  padding:.6rem;overflow-x:auto}
.fig-wrap img{display:block;width:100%;height:auto;border-radius:6px;min-width:520px}
figcaption{font-size:.83rem;color:var(--muted);margin-top:.6rem;padding-left:.2rem}
ul{padding-left:1.15rem}li{margin:.4rem 0}
b,strong{color:var(--ink)}
code{font-family:ui-monospace,"SF Mono",monospace;font-size:.86em;
  background:var(--line-2);padding:.1em .4em;border-radius:5px}
.callout{background:var(--accent-soft);border:1px solid var(--line);
  border-left:3px solid var(--accent);border-radius:10px;
  padding:.9rem 1.2rem;margin:1.5rem 0}
.callout.warn{border-left-color:var(--warn)}
.callout .t{font-weight:660;color:var(--ink);display:block;margin-bottom:.2rem}
.tag{display:inline-block;font-size:.72rem;font-weight:700;letter-spacing:.04em;
  padding:.15rem .5rem;border-radius:6px;text-transform:uppercase}
.tag.ok{background:var(--accent-soft);color:var(--coord)}
.tag.q{background:var(--accent-soft);color:var(--warn)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1.4rem 0}
@media(max-width:560px){.grid2{grid-template-columns:1fr}}
.chip{background:var(--card);border:1px solid var(--line);border-radius:10px;
  padding:.9rem 1rem}
.chip h3{margin:.1rem 0 .3rem}.chip p{margin:.2rem 0;font-size:.9rem}
.foot{margin-top:3rem;padding-top:1.2rem;border-top:1px solid var(--line);
  color:var(--muted);font-size:.82rem}
</style>
"""

HTML = STYLE + """
<div class="wrap">
  <p class="eyebrow">Leo Rover · multi-robot exploration</p>
  <h1>Two-robot collaborative exploration: what works, and an honest benchmark</h1>
  <p class="lede">A complete two-rover collaborative-SLAM exploration stack with a
  distributed frontier-coordination algorithm — plus a candid account of what the
  benchmark does and does not show.</p>
  <div class="meta mono">
    <span>ROS 2 Humble · Gazebo · WSL/GPU</span>
    <span>SLAM ×2 · Nav2 ×2 · merged map</span>
    <span>office_world &amp; a generated large world</span>
  </div>

  <h2>What was built <span class="tag ok">works</span></h2>
  <p>Two namespaced rovers, each running its own <code>slam_toolbox</code> and
  Nav2 planning on its own aligned map, with a small deterministic compositor
  stitching the per-rover maps into one shared grid. On top sits a
  <strong>distributed coordinated frontier-allocation</strong> policy
  (<code>coordination.py</code>, 6 unit tests): every cycle each rover runs the
  same greedy assignment over all rovers — information gain minus travel cost,
  plus a proximity discount — so the better-placed rover wins each frontier and
  the pair is pushed toward disjoint regions. No central node; peers are read
  from the shared TF tree and a shared claim topic.</p>

  <p><strong>The coordination demonstrably does its job.</strong> In every run
  the two rovers split the building into separate territories rather than
  crowding the same rooms — visible directly in the trajectories, and confirmed
  by the inter-rover separation being consistently larger when coordinated.</p>

  """ + fig('maps_office_world.png',
      'Final maps with rover trajectories (office_world). Left: single robot. '
      'Centre: uncoordinated pair overlaps in the middle. Right: the coordinated '
      'pair divides into separate regions (leo1 red, leo2 purple). The division '
      'is the algorithm working as intended.') + """

  """ + fig('separation_office_world.png',
      'Distance between the two rovers over time. Coordination holds them '
      'farther apart than the uncoordinated pair — it turns two robots into two '
      'separate search fronts.') + """

  <h2>The benchmark: an honest result <span class="tag q">inconclusive</span></h2>
  <p>Here is where I have to be straight. Despite the coordination working, I
  could <strong>not</strong> establish a clean "two robots explore faster than
  one" from these experiments. The coverage-over-time curves for one, two
  uncoordinated, and two coordinated rovers sit essentially on top of each
  other:</p>

  """ + fig('coverage_vs_time_office_world.png',
      'Mapped area over time (office_world, measured identically for all three, '
      'clipped to the true world extent). All three converge to the full map at '
      'similar times; the two-robot curves lead in the middle but the single '
      'robot catches up in the endgame.') + """

  <div class="callout warn">
    <span class="t">Why it is inconclusive, not "two robots are worse"</span>
    <ul>
      <li><strong>Run-to-run variance is large (~±30%).</strong> The same
      coordinated condition reached 505 m² at t=330 s in one run and 386 m² in
      another. The single-vs-two-robot difference is <em>inside</em> that noise,
      so one run per condition cannot resolve it — it needs 3–5 runs per
      condition, averaged.</li>
      <li><strong>Two-robot compute interacts with sim speed.</strong> Doubling
      SLAM + Nav2 degrades control fidelity in a way that depends on the
      real-time factor, which confounds a sim-time comparison.</li>
      <li><strong>A 20 m lidar makes one robot very efficient</strong> in open
      rooms — it maps across a room without entering, leaving little for a
      second robot to add.</li>
      <li><strong>Fixed overheads</strong> — startup clustering and one rover
      lingering to finish the last rooms while the other idles — are a large
      fraction of a short run.</li>
    </ul>
    <p>An earlier version of this report claimed "+87% from coordination." That
    was <strong>wrong</strong> — it was an artifact of cutting the uncoordinated
    runs off at a time cap before they finished. Corrected here.</p>
  </div>

  <p>The same pattern held on a purpose-built large world (30×24 m, nine rooms):
  the rovers divided it, but two coordinated rovers reached full coverage at
  t≈1000 s versus t≈810 s for a single robot — not the 2× a bigger map was
  supposed to reveal, for the same reasons above.</p>

  <h2>The engineering that made it even run</h2>
  <p>Most of the effort went into non-obvious infrastructure bugs, each of which
  silently broke the two-robot system:</p>
  <div class="grid2">
    <div class="chip"><h3>slam_toolbox /map clobber</h3>
      <p>slam publishes to the <em>absolute</em> <code>/map</code>, ignoring the
      namespace — so both rovers fought over one topic and the per-rover maps
      had no publisher. Fix: remap per rover. This was the root cause of the
      trailing rover never leaving its start area.</p></div>
    <div class="chip"><h3>GPU sat idle</h3>
      <p>The headless server rendered every lidar/camera in <em>software</em>
      (Mesa llvmpipe) because it lacked the WSL GPU libs on its path. Moving
      rendering onto the GPU (RTX 4060 Ti) took the raw sim from 0.94× to 1.73×
      real-time.</p></div>
    <div class="chip"><h3>Merged-map seams</h3>
      <p>Planning on the stitched map raised spurious walls at sub-map seams and
      blocked doorways. Fix: each rover plans on its own aligned map; the merged
      map is only for coordination and metrics.</p></div>
    <div class="chip"><h3>Metric &amp; watchdog</h3>
      <p>Coverage over-counted from odometry drift (fixed by clipping to the
      world), and uncapping the sim rate spiked the CPU and tripped a watchdog
      that killed runs (fixed by a stable rate cap).</p></div>
  </div>

  <h2>Honest conclusion</h2>
  <p>The collaborative-exploration <strong>system and algorithm work</strong>:
  two rovers build a shared map and the coordination reliably divides the
  environment between them. What is <strong>not</strong> established is a
  net exploration-<em>speed</em> advantage over a single robot in these
  simulations — the effect, if any, is small relative to run-to-run variance.
  A defensible answer would require a short statistical campaign: 3–5 runs per
  condition at a fixed real-time factor, reported with variance bands. I chose
  to stop and report this honestly rather than cherry-pick a single favourable
  run.</p>

  <p class="foot">Artifacts in reports/collab_final/ and reports/collab_big/ ·
  coverage measured identically across conditions, clipped to the world extent ·
  scripts: auto_collab_run.sh, auto_explore_run.sh, analyze_collab.py,
  make_big_world.py.</p>
</div>
"""

out = os.path.join(ROOT, 'reports', 'collab_final', 'report.html')
with open(out, 'w', encoding='utf-8') as f:
    f.write(HTML)
print('wrote', out, f'({len(HTML)//1024} KB)')
