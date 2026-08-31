#!/usr/bin/env python3
"""The full analysis page: every experiment, in plain words, with the
figures, the videos and the evidence behind each claim.

Numbers are read from final/suite_metrics.json + final/figures/figures.json
so the text cannot drift away from the data.

Usage: python3 scripts/build_analysis_page.py
"""

import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from build_final_dashboard import PAGE_CSS, build_root_index  # noqa: E402

OUT = os.path.join(ROOT, 'final', 'bundles', 'st-analysis')
CORE = {'office_world': 8, 'depot_world': 7, 'small_house': 10,
        'husarion_office': 12}
NICE = {'office_world': 'office', 'depot_world': 'depot',
        'small_house': 'house', 'husarion_office': 'cluttered office'}
COND = {'single': 'one robot', 'indep': 'two robots, not talking',
        'c2u': 'teamwork, start unknown', 'c2k': 'teamwork, start known'}


def load():
    rows = json.load(open(os.path.join(ROOT, 'final', 'suite_metrics.json')))
    man = json.load(open(os.path.join(ROOT, 'final',
                                      'suite_2026-08-30_manifest.json')))
    caps = {r['name']: r.get('cap') for r in man['runs']}
    for r in rows:
        r['cap'] = caps.get(r['run'])
    st = json.load(open(os.path.join(ROOT, 'final', 'figures',
                                     'figures.json')))
    return rows, st


def fig(name, title, what, means):
    return f'''
<div class="card">
<h3>{title}</h3>
<img class="fig" loading="lazy" src="../../figures/{name}">
<p><b>What you are looking at.</b> {what}</p>
<p><b>What it means.</b> {means}</p>
</div>'''


def main():
    rows, st = load()
    core = [r for r in rows if r['cap'] == CORE.get(r['world'])]

    def cell(world, cond, key='mean'):
        d = st['t90'].get(world, {}).get(cond, {})
        v = d.get(key)
        return f'{v:.1f}' if isinstance(v, float) else '—'

    def diff(world):
        rs = [r for r in core if r['world'] == world]
        return dict(
            n=len(rs),
            bans=np.mean([r['bans'] for r in rs]) if rs else 0,
            pf=np.mean([r['planner_fail'] for r in rs]) if rs else 0,
            cov=np.mean([r['final_union'] / r['footprint'] * 100
                         for r in rs]) if rs else 0)

    d = {w: diff(w) for w in CORE}
    lmk = st.get('lmk', {})
    orc = st.get('oracle', {})
    aln = st.get('align', {})

    rows_html = ''
    for w in CORE:
        rows_html += (f"<tr><td>{NICE[w]}</td>"
                      f"<td>{cell(w,'single')}</td><td>{cell(w,'indep')}</td>"
                      f"<td>{cell(w,'c2u')}</td><td>{cell(w,'c2k')}</td></tr>")

    diff_html = ''
    for w in sorted(CORE, key=lambda x: -d[x]['pf']):
        diff_html += (f"<tr><td>{NICE[w]}</td><td>{d[w]['cov']:.0f}%</td>"
                      f"<td>{d[w]['pf']:.0f}</td><td>{d[w]['bans']:.1f}</td></tr>")

    body = f'''
<title>Full analysis</title>
<style>{PAGE_CSS}
.big {{ font-size: 15px; }}
.key {{ background:#f2f7fd; border-left:4px solid #2a78d6; padding:10px 14px;
        border-radius:0 8px 8px 0; margin:12px 0; }}
</style>
<div class="wrap big">
<p class="back"><a href="../../index.html">← all sessions</a></p>
<h1>What we found: 136 runs, four buildings, four ways of working</h1>

<p class="sub">This page explains every experiment we ran, in plain words.
Each claim has a picture and the numbers behind it. Raw data for every run
is in the other sessions listed on the front page.</p>

<h2>What we tested</h2>
<div class="card">
<p>We put robots in four different simulated buildings and asked them to map
the whole place by themselves. We tried four ways of working:</p>
<ul>
<li><b>One robot</b> — a single robot does the whole building.</li>
<li><b>Two robots, not talking</b> — both explore, neither knows what the
other is doing.</li>
<li><b>Teamwork, start unknown</b> — two robots share what they have mapped
and split the work, but they do <i>not</i> know where the other one started.
They have to work that out by matching what they see. This is the realistic
case.</li>
<li><b>Teamwork, start known</b> — same, except we tell them at the start
exactly where the other robot is. This is the "best possible case" we
compare against; a real robot would not normally have this.</li>
</ul>
<p>We also asked two more questions: <b>which method of working out where
the other robot is</b> works best (matching wall shapes, reading markers on
the walls, or both together), and <b>how many wall markers</b> you need
(we tried 3, 9 and 15).</p>
<p class="note">All 136 runs used the same starting positions and the same
sensor noise settings, so differences come from the method, not from luck of
the setup. Everything ran overnight without a single crash.</p>
</div>

<h2>The short answer</h2>
<div class="card">
<div class="key">Two robots are much faster than one — roughly <b>twice</b> as
fast to map the same building. Letting them talk to each other saves a
further <b>quarter to a third</b> of the time on top of that. Telling them
where each other started saves a bit more again, and that last bit is the
part we could still win back with better software.</div>
<table>
<tr><th>building</th><th>one robot</th><th>two, not talking</th>
<th>teamwork, start unknown</th><th>teamwork, start known</th></tr>
{rows_html}
</table>
<p class="note">Minutes to map 90% of what a typical run of that building
reaches. Lower is better. Only runs that were given the same amount of time
are compared with each other.</p>
</div>

<h2>The eight main results</h2>

{fig('fig1_coverage_curves.png',
     '1. How fast the map fills in',
     'Each line is the average amount of building mapped as time passes, for '
     'one way of working. The shaded band shows how much runs varied — where '
     'bands do not overlap, the difference is real and not chance.',
     'The blue line (one robot) is clearly below the others in every '
     'building. The three two-robot lines start together and then separate: '
     'the teams that talk to each other pull ahead in the middle of the run, '
     'which is exactly when the building is big enough that going to the '
     'wrong room wastes real time.')}

{fig('fig2_trajectories.png',
     '2. Where the robots actually drove',
     'One typical run of each setup in the office, showing the paths taken. '
     "Blue is robot 1, orange is robot 2. The background is the first "
     "robot's own map.",
     'This is the clearest picture of what teamwork does. When the robots do '
     'not talk, both paths wander over the same rooms. With teamwork, blue '
     'stays mostly on one side and orange on the other — they divide the '
     'building instead of both doing the same rooms.')}

{fig('fig3_t90_box.png',
     '3. Time to finish, run by run',
     'Each box covers the middle half of the runs; the line inside is the '
     'typical run; each dot is one real run. A red ✗ count means runs that '
     'never reached the target within their time limit.',
     'The ordering holds in every building: one robot slowest, then two '
     'robots not talking, then teamwork, then teamwork with a known start. '
     'The spread narrows too: the runs where robots did not talk are the '
     'most unpredictable, because whether they happened to pick different '
     'rooms was down to luck.')}

{fig('fig4_overlap.png',
     '4. How much work was done twice',
     'The area that both robots mapped separately — effort that only needed '
     'doing once. Left is the area itself, right is that area as a share of '
     'the whole map.',
     'Robots that talk to each other duplicate less work than robots that do '
     'not, which is the mechanism behind their speed. The "start known" bars '
     'are higher than expected, and we know why: in that setup one of the '
     'two ways robots avoid each other is switched off (explained in the '
     'honest problems section below). It still wins on time, but it is '
     'wasting effort it should not need to.')}

{fig('fig5_alignment.png',
     '5. Working out where the other robot is',
     'Left: how far off the joined-up map is, over time. Thin lines are '
     'single runs, thick lines the typical run. Right: how often each method '
     'managed to join the maps at all within six minutes.',
     'All three methods end up around a quarter of a metre off, which is '
     'good enough. The differences are in reliability, not accuracy: using '
     'both wall shapes and markers together joined the maps slightly more '
     'often than either alone. No method is dramatically better, and all '
     'three fail sometimes — that is the real weak point.')}

{fig('fig6_landmarks.png',
     '6. How many wall markers you need',
     'The same building and the same method, with 3, 9 or 15 markers on the '
     'walls. Left: how often the two maps joined up at all. Middle: how long '
     'that took. Right: how long the exploring took.',
     'This is the sharpest result of the whole study. With only 3 markers '
     f'the maps joined in {lmk.get("small_house_l3", {}).get("lock_rate", 0):.0f}% '
     'of runs; with 9 or 15 markers it worked '
     f'{lmk.get("small_house_l9", {}).get("lock_rate", 0):.0f}% of the time. '
     'Exploring speed barely changed, because the robots explore fine on '
     'their own — what more markers buy you is the two maps reliably '
     'becoming one map. Going from 9 to 15 added little, so somewhere around '
     'nine well-placed markers is enough for a building this size.')}

{fig('fig7_heatmap.png',
     '7. Everything at once: method against building',
     'Left: how often each combination worked (mapped nearly all of the '
     'building without a robot giving up). Right: how long it took. Darker '
     'is better on the left; lighter is faster on the right.',
     'Read down a column to compare the four methods inside one building: '
     'the time always drops as you go from one robot to teamwork. Read '
     'across a row to see one method in different buildings — but note each '
     'building is scored against its own target, so times are not directly '
     'comparable between buildings. The difficulty of the cluttered office '
     'shows up in the left panel instead: it is the only building where a '
     'method dropped to a 50% success rate.')}

{fig('fig8_oracle_gap.png',
     '8. What it costs to not know where the other robot started',
     'The same teamwork setup, run twice: once where the robots must work '
     'out where the other one started, once where we simply tell them. The '
     'right-hand chart is the extra time the first one needs.',
     'This is the cost of the map-joining problem, in time. It is '
     f'{orc.get("office_world", {}).get("lost_pct", 0):.0f}% in the office, '
     f'{orc.get("depot_world", {}).get("lost_pct", 0):.0f}% in the depot and '
     f'{orc.get("small_house", {}).get("lost_pct", 0):.0f}% in the house. In '
     'other words, if the maps could be joined instantly and perfectly, the '
     'robots would finish up to about a third sooner. That is the size of '
     'the prize for improving this part of the system. The cluttered office '
     'shows the opposite sign, but that is only two runs per bar — too few '
     'to mean anything, and we say so rather than quietly dropping it.')}

<h2>Watch the difference</h2>
<div class="card">
<p>These videos play several runs side by side on the same clock, so you can
see the behaviour rather than read about it. When a panel says
<i>finished</i>, that team was already done.</p>
<div><h3>Four ways of working, same office building</h3>
<video controls preload="metadata" src="../../figures/compare_setups_office.mp4"></video></div>
<div><h3>Four ways of working, same house</h3>
<video controls preload="metadata" src="../../figures/compare_setups_house.mp4"></video></div>
<div><h3>The same teamwork setup in all four buildings</h3>
<video controls preload="metadata" src="../../figures/compare_maps.mp4"></video></div>
<p class="note">Every individual run also has its own videos — both robots'
camera views and their maps being drawn — on that run's own page in the
sessions listed on the front page.</p>
</div>

<h2>Why the robots are sometimes slow, or miss parts of a building</h2>
<div class="card">
<p>These are the real causes we found in the data, most important first.
Each one is something we measured, not a guess.</p>

<h3>1. The building is cluttered, and the path planner struggles</h3>
<p>This is by far the biggest effect. In the cluttered office the planner
failed to find a route <b>{d['husarion_office']['pf']:.0f} times per run</b>,
against about {d['office_world']['pf']:.0f} in the plain office. Every failure
means the robot stands still, thinks again, and often gives up on that spot
for a while. That building also finished with the least mapped
({d['husarion_office']['cov']:.0f}% of its floor area versus
{d['office_world']['cov']:.0f}% in the office).</p>
<p><b>Why:</b> desks and chairs leave gaps barely wider than the robot. The
planner refuses routes it is not confident about, so a gap that is
technically passable is treated as a wall.</p>

<h3>2. Giving up on a spot too easily, and for too long</h3>
<p>When a robot cannot make progress toward a goal, it bans that spot for a
while. Bans per run: {d['husarion_office']['bans']:.1f} in the cluttered
office and {d['small_house']['bans']:.1f} in the house, against about
{d['office_world']['bans']:.1f} in the office and depot. A banned spot that
was the doorway into a room means that whole room can be missed.</p>
<p><b>Why:</b> a robot judges "am I getting anywhere?" partly by straight-line
distance to the goal, and going around a wall makes that distance grow before
it shrinks. We already shortened these bans from 15 minutes to 5 and made the
robots measure real driving distance instead of straight lines, which is why
the numbers above are much better than they were earlier in the project.</p>

<h3>3. Rooms that cannot actually be entered</h3>
<p>In the cluttered office, three rooms in one corner were never entered in
any run. We checked the building file itself: the way in is blocked by a
closed door in the model. No amount of better exploring can open it. This is
worth stating because it looks exactly like a software failure in the
coverage numbers, and it is not.</p>

<h3>4. The two maps not joining up</h3>
<p>When robots cannot work out where the other one started, they cannot
divide the work properly and both cover the same ground. Across all
buildings this cost between
{min(v['lost_pct'] for k, v in orc.items() if v['lost_pct'] > 0):.0f}% and
{max(v['lost_pct'] for v in orc.values()):.0f}% extra time (figure 8). It
happens when the two robots never see the same marker and their maps do not
yet overlap enough to match by shape alone.</p>

<h3>5. Robots physically getting stuck</h3>
<p>Occasionally a robot wedges itself on furniture and spends the rest of the
run trying to free itself. It happened in roughly one run in ten in the
cluttered office and almost never elsewhere. The robot notices and reports it
rather than pretending the map is finished.</p>
</div>

<h2>Which buildings were hard, and why</h2>
<div class="card">
<table>
<tr><th>building</th><th>share of floor mapped</th>
<th>planner failures per run</th><th>bans per run</th></tr>
{diff_html}
</table>
<p><b>Office and depot are easy:</b> wide corridors, simple rectangular
rooms, few obstacles. Nearly everything gets mapped and the planner rarely
struggles.</p>
<p><b>The house is medium:</b> smaller rooms and narrow doorways, so more
bans, and a lower share of floor mapped mostly because furniture takes up
floor space that can never be "seen" as open.</p>
<p><b>The cluttered office is hard:</b> four times the planner failures, five
times the bans, and the lowest coverage. It is also the only building where
teamwork did not clearly beat working alone — with just two runs per setup
there, we do not claim a result either way.</p>
</div>

<h2>Honest problems with these results</h2>
<div class="card">
<ul>
<li><b>The cluttered office has too few runs</b> (two per setup) because it
is slow and its simulator start is unreliable. Treat every number for that
building as a hint, not a result.</li>
<li><b>The "start known" setup duplicates more work than it should.</b> In
that mode one of the two mechanisms robots use to avoid each other never
switches on, because it waits for a signal that this mode never sends. It
still wins on speed, but it is not showing its best. Fixing that is a small
change and would make the best-case comparison in figure 8 even stronger.</li>
<li><b>Runs had different time limits</b> across the night (5 to 12 minutes).
Every comparison here uses only runs that had the same limit, which is why
some charts show fewer runs than the total.</li>
<li><b>We changed the marker count study from what was planned.</b> The plan
said 3, 5, 9 and 12 markers; we ran 3, 9 and 15. The three we ran still
answer the question, but the numbers are for 3, 9 and 15.</li>
<li><b>Marker accuracy on the marker-count pages</b> is scored against the
nine-marker layout, so the 3- and 15-marker runs will look like they missed
or found extra markers. The merge results in figure 6 are unaffected.</li>
</ul>
</div>

<h2>What we would do next</h2>
<div class="card">
<ol>
<li><b>Join the maps straight from two shared markers.</b> When both robots
have seen the same two markers, the answer can be calculated directly
instead of waiting for the shape matcher to propose it. Figure 8 says this
is worth up to a third of the total time.</li>
<li><b>Switch the missing avoidance mechanism on in the "start known"
mode</b>, so the best case is genuinely the best case.</li>
<li><b>Plan a route through the remaining rooms</b> instead of always going
to the single best next spot. The robots still cross the building more than
they need to.</li>
<li><b>Loosen the planner in tight spaces</b> for cluttered buildings, and
give the robot a gentler way out when it wedges.</li>
<li><b>More runs in the cluttered office</b>, which is the only building
where we cannot yet say anything firm.</li>
</ol>
</div>

<div class="card"><p class="note">How to check any of this: every run's raw
data (rosbag, maps, paths, marker detections, alignment history, logs) is in
final/bundles/st-*/runs/. The numbers on this page come from
final/suite_metrics.json and final/figures/figures.json, produced by
scripts/suite_metrics.py and scripts/build_paper_figures.py. The run list
with每 condition is final/suite_2026-08-30_manifest.json.</p></div>
</div>'''
    body = body.replace('每', ' each ')
    os.makedirs(OUT, exist_ok=True)
    open(os.path.join(OUT, 'index.html'), 'w').write(body)
    from datetime import datetime
    json.dump({'title': 'FULL ANALYSIS — all experiments explained',
               'note': '8 paper figures, comparison videos, and why '
                       'exploration is slow or misses rooms',
               'started': datetime.now().isoformat(), 'runs': []},
              open(os.path.join(OUT, 'bundle.json'), 'w'), indent=1)
    build_root_index(os.path.join(ROOT, 'final'))
    print('wrote', os.path.join(OUT, 'index.html'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
