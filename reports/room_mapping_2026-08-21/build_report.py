"""Assemble the run-debrief HTML with evidence images inlined as data URIs."""
import base64
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def uri(name):
    with open(os.path.join(HERE, name), "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


IMG = {k: uri(k) for k in (
    "timeline.png", "map_evolution.png", "final_costmaps.png", "video_strip.png")}
IMG["run2_timeline"] = uri("run2/timeline2.png")
IMG["run2_final"] = uri("run2/run_2026-08-21_2/render/debug_00230.png")

html = """<title>The 170-Second Livelock</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@500;600;700&family=Source+Sans+3:ital,wght@0,400;0,600;1,400&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
:root {
  --bg: #F5F6F7; --surface: #FFFFFF; --ink: #1C2429; --muted: #5A6A73;
  --line: #D9DfE3; --accent: #C63B2A; --data: #2E7F8A; --warn: #B07E1F;
  --good: #3A7D44; --chip-bg: #EDF0F2;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --bg: #14181D; --surface: #1C2229; --ink: #E7ECEF; --muted: #92A1AB;
    --line: #313B44; --accent: #E05A47; --data: #55AEBB; --warn: #D3A445;
    --good: #6FB579; --chip-bg: #242C34;
  }
}
:root[data-theme="dark"] {
  --bg: #14181D; --surface: #1C2229; --ink: #E7ECEF; --muted: #92A1AB;
  --line: #313B44; --accent: #E05A47; --data: #55AEBB; --warn: #D3A445;
  --good: #6FB579; --chip-bg: #242C34;
}
* { box-sizing: border-box; }
body {
  background: var(--bg); color: var(--ink); margin: 0;
  font-family: "Source Sans 3", "Segoe UI", sans-serif;
  font-size: 17px; line-height: 1.55;
}
main { max-width: 880px; margin: 0 auto; padding: 40px 24px 80px; }
h1, h2, h3 { font-family: Archivo, "Segoe UI", sans-serif; text-wrap: balance; }
h1 { font-size: 2.1rem; font-weight: 700; line-height: 1.15; margin: 8px 0 4px; }
h2 { font-size: 1.35rem; font-weight: 600; margin: 44px 0 10px; }
h3 { font-size: 1.05rem; font-weight: 600; margin: 24px 0 6px; }
p { max-width: 68ch; }
.eyebrow {
  font-family: "IBM Plex Mono", monospace; font-size: 0.72rem;
  letter-spacing: 0.14em; text-transform: uppercase; color: var(--data);
}
.verdict {
  border-left: 4px solid var(--accent); background: var(--surface);
  padding: 14px 20px; margin: 22px 0; border-radius: 0 6px 6px 0;
}
.verdict p { margin: 6px 0; }
.stats { display: flex; flex-wrap: wrap; gap: 10px; margin: 18px 0 6px; }
.stat {
  background: var(--surface); border: 1px solid var(--line); border-radius: 6px;
  padding: 8px 14px; min-width: 118px;
}
.stat .v {
  font-family: "IBM Plex Mono", monospace; font-size: 1.25rem; font-weight: 500;
  font-variant-numeric: tabular-nums; display: block;
}
.stat .k { font-size: 0.75rem; color: var(--muted); letter-spacing: 0.05em; text-transform: uppercase; }
.stat.bad .v { color: var(--accent); }
.stat.warn .v { color: var(--warn); }
.stat.good .v { color: var(--good); }
figure { margin: 18px 0; }
.scroll { overflow-x: auto; border: 1px solid var(--line); border-radius: 6px; background: #fff; }
.scroll img { display: block; max-width: 100%; height: auto; }
.scroll.wide img { max-width: none; width: 1400px; }
figcaption { font-size: 0.85rem; color: var(--muted); margin-top: 6px; max-width: 75ch; }
ol.chain { list-style: none; counter-reset: c; padding: 0; margin: 18px 0; }
ol.chain li {
  counter-increment: c; position: relative; padding: 12px 16px 12px 58px;
  background: var(--surface); border: 1px solid var(--line); border-radius: 6px;
  margin-bottom: 10px; max-width: 74ch;
}
ol.chain li::before {
  content: counter(c); position: absolute; left: 16px; top: 12px;
  font-family: "IBM Plex Mono", monospace; font-weight: 500; font-size: 1.1rem;
  color: var(--accent);
}
code, .mono {
  font-family: "IBM Plex Mono", monospace; font-size: 0.86em;
  background: var(--chip-bg); padding: 1px 5px; border-radius: 4px;
}
table { border-collapse: collapse; width: 100%; font-size: 0.95rem; }
th, td { text-align: left; padding: 8px 12px; border-bottom: 1px solid var(--line); vertical-align: top; }
th { font-family: Archivo, sans-serif; font-size: 0.8rem; letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted); }
.sev { font-family: "IBM Plex Mono", monospace; font-size: 0.78rem; padding: 2px 8px; border-radius: 10px; white-space: nowrap; }
.sev.p1 { background: var(--accent); color: #fff; }
.sev.p2 { background: var(--warn); color: #fff; }
.sev.p3 { background: var(--muted); color: #fff; }
.footer-note { color: var(--muted); font-size: 0.85rem; border-top: 1px solid var(--line); margin-top: 48px; padding-top: 14px; }
a { color: var(--data); }
</style>
<main>
<span class="eyebrow">Leo Rover 4 &middot; autonomous exploration debrief &middot; 2026-08-21 13:16&ndash;13:21</span>
<h1>The 170-Second Livelock</h1>
<p>Why the rover parked in the middle of the room, spun in place, and never
reached the doorway &mdash; reconstructed from the on-board recorder
(map + costmaps + path + frontiers + Nav2 goals + camera, 2&nbsp;s cadence).</p>

<div class="verdict">
<p><strong>Verdict.</strong> The robot was not lost and the sensors were fine.
It drove 2.5&nbsp;m, reached the centroid of the <em>last visible frontier</em>, and
then sat inside its own goal tolerance for 170 seconds while
<strong>explore_lite re-sent the same goal ~7&times; per second and Nav2 instantly
reported each one SUCCEEDED</strong> &mdash; 1,142 goals, 1,181 instant successes in a
5-minute run. A motionless robot never crosses slam_toolbox&rsquo;s minimum-travel
gate, so no scan was integrated, the map stayed frozen, and the frontier under
its wheels could never be cleared. The loop broke on its own at t&asymp;280&nbsp;s;
the map instantly doubled, 13 fresh frontiers appeared, and the rover set off
toward the doorway gap &mdash; at which point the run had to be stopped
(battery sagging toward 11&nbsp;V).</p>
</div>

<div class="stats">
<div class="stat"><span class="v">324 s</span><span class="k">recorded</span></div>
<div class="stat"><span class="v">4.0 m</span><span class="k">distance</span></div>
<div class="stat"><span class="v">19.6 m&sup2;</span><span class="k">mapped</span></div>
<div class="stat bad"><span class="v">1,142</span><span class="k">goals sent</span></div>
<div class="stat bad"><span class="v">1,181</span><span class="k">instant succeeds</span></div>
<div class="stat good"><span class="v">4</span><span class="k">aborts</span></div>
<div class="stat warn"><span class="v">11.0 V</span><span class="k">battery at stop</span></div>
</div>

<h2>The run at a glance</h2>
<figure>
<div class="scroll"><img src="%%timeline%%" alt="Four stacked time series: map area, distance travelled, frontier count, and goal-event rate"></div>
<figcaption>t=0 is recorder start; exploration begins at t&asymp;88&nbsp;s. All four
signals flatline together during t&asymp;110&ndash;280&nbsp;s: map area frozen at
10.6&nbsp;m&sup2;, distance stuck at 2.6&nbsp;m, exactly one frontier (under the robot),
and ~200 goal events per 10&nbsp;s hammering Nav2. At t&asymp;280 the first
successfully integrated scan doubles the map and exploration resumes.</figcaption>
</figure>

<h2>Failure chain</h2>
<ol class="chain">
<li><strong>The rover reached the centroid of the only frontier big enough to
chase</strong> (&ge;0.5&nbsp;m). Its pose landed inside the goal checker&rsquo;s xy + yaw
tolerance.</li>
<li><strong>Every replan became an instant success.</strong> explore_lite replans
every 5&nbsp;s, but each <code>NavigateToPose</code> goal completed in 50&ndash;80&nbsp;ms
(&ldquo;Reached the goal!&rdquo; with zero motion), so its progress-timeout blacklist
logic never fired &mdash; blacklisting only triggers on goals that <em>fail to make
progress</em>, not goals that succeed vacuously.</li>
<li><strong>A motionless robot starves SLAM.</strong> slam_toolbox only integrates a
scan after &ge;0.05&nbsp;m of travel or 0.05&nbsp;rad of turn. Standing perfectly
still, it integrated nothing, so the map &mdash; and the phantom frontier &mdash;
never changed. The map is the input to explore_lite, closing the loop.</li>
<li><strong>Scan rejection made it worse.</strong> The RPLidar emits a variable
number of points per revolution (491&ndash;512 observed); karto templates the
laser on the first scan (513) and <em>silently rejects every scan that differs</em>
(&ldquo;LaserRangeScan contains 507 range readings, expected 513&rdquo; &mdash; 80 logged
rejects). That is why the map is thin and ray-speckled, and why the corridor
beyond the doorway &mdash; visible to the lidar from t&asymp;96&nbsp;s as those long rays
&mdash; only consolidated into free space at t&asymp;280&nbsp;s.</li>
</ol>
<p>The &ldquo;dumb rotating&rdquo; near the end was the loop finally breaking: a slightly
different goal orientation triggered a real rotation, the 0.05&nbsp;rad gate
tripped, one scan got accepted, and the map caught up all at once.</p>

<h2>Map evolution</h2>
<figure>
<div class="scroll"><img src="%%map_evolution%%" alt="Six SLAM map snapshots with robot path, frontiers, and goals"></div>
<figcaption>Purple = free, yellow = walls, grey = unknown. White line: path.
Red stars: frontier centroids. Yellow &times;: current Nav2 goal. Frames 2&ndash;5
(t=96&ndash;276&nbsp;s) are nearly identical &mdash; the freeze &mdash; with the lone frontier
star sitting on the robot. At t=322&nbsp;s the map has doubled, 13 frontiers
appeared, and the rover is heading east toward the doorway.</figcaption>
</figure>

<h2>Final costmaps</h2>
<figure>
<div class="scroll"><img src="%%final_costmaps%%" alt="Final SLAM map, global costmap and local costmap side by side"></div>
<figcaption>The global costmap&rsquo;s inflated ring encloses the room except one
gap at &asymp;(4.3, 1.8)&nbsp;m &mdash; the doorway. The rover&rsquo;s final goal (yellow &times;)
was exactly that gap; the local costmap shows the narrow free channel it was
threading when the run was stopped. The camera layer (robust cloud_filter,
4.6&nbsp;Hz) contributed no phantom obstacles &mdash; the costmaps are clean.</figcaption>
</figure>

<h2>What the camera saw</h2>
<figure>
<div class="scroll wide"><img src="%%video_strip%%" alt="Five low-resolution camera frames across the run"></div>
<figcaption>320&times;180 snapshots at 1&nbsp;Hz (323 frames saved). Left to right:
launch pose, driving east past the whiteboard, then three near-identical views
during the livelock &mdash; the rover staring at the same shelf for three minutes
&mdash; and finally the turn toward the doorway just before shutdown.</figcaption>
</figure>

<h2>Contributing factors</h2>
<p><strong>CPU saturation for the first ~25 minutes after boot.</strong> Load peaked at
23 on 6 cores: a leftover experiment node (<code>color_detector</code>, one full
core), Ubuntu&rsquo;s <code>update-manager</code>/<code>unattended-upgrades</code> (one core),
and the goal-storm itself (planner + behavior-tree churn at 7&nbsp;Hz). Both
parasites were killed mid-run; they return on every reboot.</p>
<p><strong>Battery.</strong> 11.27&nbsp;V at preflight, 11.0&nbsp;V twenty minutes later
&mdash; the pack was near its floor before exploration even started, which is why
the run was cut at the first good stopping point. Charge before the next
attempt.</p>
<p><strong>Not guilty this time:</strong> the camera pipeline. The imported robust
mode needed two on-rover fixes before it would run at all (a crash on the
Jetson&rsquo;s 16-byte-padded point clouds, and a TF listener starved by 30&nbsp;Hz
cloud deserialization), but once up it filtered at 4.6&nbsp;Hz and the costmaps
show no phantom obstacles &mdash; the simplified room did its job.</p>

<h2>Fixes for the next run</h2>
<table>
<thead><tr><th>Priority</th><th>Fix</th><th>Why it works</th></tr></thead>
<tbody>
<tr><td><span class="sev p1">P1</span></td>
<td>Normalize scan length before SLAM: a tiny relay that pads/truncates every
scan to exactly 513 ranges (or enable the driver&rsquo;s angle-compensate mode).</td>
<td>Makes every revolution integrable instead of a lucky few &mdash; denser map,
frontiers clear the moment the robot looks at them, and the freeze becomes
much harder to enter.</td></tr>
<tr><td><span class="sev p1">P1</span></td>
<td>Break the vacuous-success loop in explore_lite: blacklist a frontier after
N consecutive SUCCEEDED goals with no map change (we build m-explore from
source, ~15 lines), or a watchdog that commands a 0.3&nbsp;m nudge when
map+pose are static while goals succeed.</td>
<td>Attacks the livelock directly &mdash; either the frontier is dropped or the
nudge trips SLAM&rsquo;s travel gate and the map updates.</td></tr>
<tr><td><span class="sev p2">P2</span></td>
<td>Boot hygiene in <code>start_stack.sh</code>: kill <code>color_detector</code>,
stop <code>unattended-upgrades</code>, verify load &lt; 6 before launch.</td>
<td>The goal-storm was surviving on a saturated CPU; headroom keeps TF fresh
and the controller smooth.</td></tr>
<tr><td><span class="sev p2">P2</span></td>
<td>Charge the battery; treat &lt;11.2&nbsp;V at preflight as no-go.</td>
<td>This run ended exactly when exploration got interesting.</td></tr>
<tr><td><span class="sev p3">P3</span></td>
<td>Persist the camera params (<code>pointcloud__neon_.enable</code>,
<code>decimation_filter</code> magnitude 4) in the leo-ros launch instead of
per-session <code>ros2 param set</code>.</td>
<td>Removes two silent-failure preflight steps.</td></tr>
</tbody>
</table>

<h2>Run 2 (same day, fixes applied): doorway found</h2>
<div class="verdict">
<p><strong>With the scan normalizer, the explore patch, and tighter inflation,
the second run mapped 82.5&nbsp;m&sup2; (8&times; run 1), transited the doorway, and
pushed 10+&nbsp;m up the corridor</strong> before being stopped by hand at 470&nbsp;s.
No livelock, no goal storm (47 goals vs 1,142). The remaining complaint is
pace: 32% of the run driving (pinned at the 0.10&nbsp;m/s regulated floor), 18%
rotating in place, 50% stationary between path refreshes.</p>
</div>

<div class="stats">
<div class="stat good"><span class="v">82.5 m&sup2;</span><span class="k">mapped (8&times; run 1)</span></div>
<div class="stat good"><span class="v">16.7 m</span><span class="k">distance</span></div>
<div class="stat good"><span class="v">0</span><span class="k">karto rejects</span></div>
<div class="stat good"><span class="v">47</span><span class="k">goals (vs 1,142)</span></div>
<div class="stat warn"><span class="v">0.10 m/s</span><span class="k">cruise (want 0.26)</span></div>
<div class="stat warn"><span class="v">50%</span><span class="k">time stationary</span></div>
</div>

<figure>
<div class="scroll"><img src="%%run2_final%%" alt="Run 2 final state: dense SLAM map, clean costmaps, robot in corridor"></div>
<figcaption>t=460&nbsp;s: dense continuous walls (red = occupied cells), room fully
mapped, robot in the corridor heading for the frontier at (16, 10). The
penalty bands now hug real walls instead of blooming around speckles.</figcaption>
</figure>

<figure>
<div class="scroll"><img src="%%run2_timeline%%" alt="Run 2 map growth, speed and yaw-rate time series"></div>
<figcaption>Speed samples cluster at zero and at the 0.10&nbsp;m/s regulated
floor; the desired 0.26&nbsp;m/s is never reached. The yaw-rate track shows the
constant rotate-in-place interruptions.</figcaption>
</figure>

<h3>Why it crawled, and the fix</h3>
<p>The controller stack is a RotationShim wrapping RegulatedPurePursuit. As the
map grows, the behavior tree hands the controller a refreshed path every
~1.4&nbsp;s (338 times this run); whenever the new path&rsquo;s initial heading
differed by &gt;0.5&nbsp;rad the shim <em>stopped the robot</em> and rotated at
0.6&nbsp;rad/s before driving again &mdash; the &ldquo;0.5&nbsp;m, then rotate, then again&rdquo;
pattern. Meanwhile cost-regulated speed scaling pinned cruise speed to its
0.10&nbsp;m/s floor whenever the robot was inside any inflation band &mdash; which in
a room is nearly always. Params changed for the next run:
<code>angular_dist_threshold</code> 0.5&rarr;1.0,
<code>rotate_to_heading_angular_vel</code> 0.6&rarr;1.2 (accel 1.2&rarr;2.0),
<code>use_cost_regulated_linear_velocity_scaling</code> off,
explore <code>planner_frequency</code> 0.2&rarr;0.1 (half the goal churn). Expected:
cruise at 0.26&nbsp;m/s, rotations rare and twice as fast.</p>

<p class="footer-note">Data: <span class="mono">reports/room_mapping_2026-08-21/</span>
(163 recorder frames, 323 camera frames, events.jsonl, stack + explore logs, saved map
<span class="mono">maps/run_2026-08-21_1</span>); same archive on the rover at
<span class="mono">~/leo_nav2_ws/runs/run_2026-08-21_1</span>. Stack: ivan-branch robust
overlay (tip 2f57b33, imported as ed30f34) + two uncommitted cloud_filter fixes made today.</p>
</main>
"""

for k, v in IMG.items():
    html = html.replace("%%" + k.replace(".png", "") + "%%", v)

out = os.path.join(HERE, "report.html")
with open(out, "w", encoding="utf-8") as f:
    f.write(html)
print("wrote", out, len(html) // 1024, "KB")
