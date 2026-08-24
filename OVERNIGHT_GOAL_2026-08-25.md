# Overnight goal — distributed shared mapping, merged with or without markers

**Branch:** `feat/multi-robot-integration`
**Deadline:** 09:00 Europe/Berlin, 2026-08-25. Report written by 08:30.
**Ledger:** `reports/night_2026-08-25/PROGRESS.md` — created in Phase 0, appended
every tick. **The ledger is the source of truth for where you are.** Lose
context, read it first, resume from it. Never restart a phase it marks COMPLETE.

---

## Mission

Two rovers start in different rooms **knowing nothing about each other** — no
shared frame, no known offsets, no ground-truth odometry. Each builds its own
map. As soon as their maps overlap enough to be related, they merge — **markers
are a convenience, not a requirement.** Recovering the transform between two
occupancy grids is a rotation-translation problem and must be solved as one.

Once merged, each rover holds an *enriched* map: it can see what the other has
already covered, and stops going there. That is the whole point — the shared map
is not a deliverable, it is an input to better exploration.

**It must be distributed.** Each rover merges the peer's map locally and acts on
its own copy. No central merger, no laptop as a single point of failure. A rover
that loses the network keeps exploring on what it already knows.

And it must survive contact with the lab tomorrow: code deployed to two physical
rovers, markers taped to office walls, both started, no babysitting.

### Explicitly NOT the goal

- Beating the uncoordinated baseline. Measure it, report it, move on.
- Seeds. One per configuration. Re-run only what is *broken* (see below).
- Three rovers, item search, soak tests.

---

## The single most important instruction

**Iterate offline.** `scripts/merge_benchmark.py` scores any merging method
against **ten real saved map pairs** with exact known ground truth, in about a
minute:

```bash
python3 scripts/merge_benchmark.py --method mymodule:match
```

A method takes `(grid1, info1, grid2, info2)` and returns `(dx, dy, yaw_rad)`
mapping map2 into map1, or `None` to abstain. The naive correlative baseline
already in `scripts/_baseline_matcher.py` scores **0/10**, with 180° flips on
every depot and office pair — reproduce that first so you know the harness works.

**Do not start a simulation to test a merging change.** A sim run costs 25
minutes and answers one question; the benchmark costs one minute and answers
ten. Simulate only to test things the benchmark cannot see: exploration
behaviour, timing, the distributed plumbing, the real-launch path.

---

## Phase 0 — get oriented (target ≤ 30 min)

1. `colcon build --symlink-install`, run the unit tests. They pass at 61/61 on
   the current tip; if not, that is the night's first finding.
2. Create `reports/night_2026-08-25/PROGRESS.md` and write the plan into it.
3. `python3 scripts/merge_benchmark.py --method _baseline_matcher:match` →
   expect **0/10**. Confirms the benchmark, the maps, and the ground truth.
4. Confirm the GPU: start any run and check
   `/root/.ignition/rendering/ogre2.log` says
   `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)`. Anything mentioning
   llvmpipe means software rendering and a 6× slowdown.

**Gate:** build green, tests green, benchmark reproduces 0/10, GPU confirmed.

---

## Phase 1 — merge two maps without markers (target ≤ 3 h, offline)

The hard problem, and the one the whole night rests on. **No simulation in this
phase.**

The existing `map_based_aligner` fails the same way every time: depot and office
are rectilinear boxes full of rectilinear obstacles, so rotating one map by 90°
or 180° overlays the other almost perfectly. Measured across four runs it
proposed 179.1°, 91.0°, 90.0° and 179.9° errors — **at confidence 0.60 to 0.75**.
Confidence built on overlap score cannot see this, because the wrong hypothesis
genuinely scores well.

What is likely to work, in rough order of expected value:

- **Score every rotation hypothesis and compare the best two.** In a symmetric
  room the best and runner-up score nearly the same. Requiring a *margin*
  between them is the single most direct test for "this match is ambiguous",
  and it converts a confident-wrong answer into an honest abstention.
- **Branch-and-bound over rotation** (Cartographer-style) rather than a coarse
  sweep, so the search is both complete and fast enough to be exhaustive.
- **Match free space, not walls.** Free space in these worlds is markedly less
  symmetric than the wall skeleton.
- **Structural descriptors** — corner and junction detection on the grid,
  matched with RANSAC — which do not care about global symmetry at all.
- **Use the overlap region only.** Scoring cells one map has never seen rewards
  hypotheses that hide disagreement in unknown space.

**Abstaining is a pass.** A merge that refuses to commit is worth far more than
one that is confidently 180° wrong, because everything downstream — the shared
map, the exploration decisions, the lab session — inherits the error.

**Gate — all four:**
1. **≥ 7 of 10** benchmark pairs within **0.5 m and 10°**.
2. **Zero** pairs wrong by more than 1 m or 15° *without abstaining*. This is the
   hard one and it matters more than the pass count.
3. Runs in **under 5 s** per pair on this machine, so it can run live.
4. Works with **markers switched off entirely** — prove it by scoring with the
   tag pipeline disabled.

Then, and only then, wire it in and confirm with **one** simulated run.

---

## Phase 2 — the merged map must change what the rovers do (target ≤ 2 h)

Today each rover detects frontiers on **its own** `/leo{i}/map` only. So even
after a successful merge, neither rover benefits: it will happily drive across a
room the other finished ten minutes ago. Fix that, because it is the actual
payoff the mission asks for.

1. Each rover explores against the **merged** map once one exists, and falls
   back to its own the moment the merge is withdrawn. Degrading must be
   seamless — no stalling, no thrashing between the two.
2. Frontier detection must treat peer-covered space as covered.
3. Keep the frontier bounds, border margin and obstacle clearance already in
   place — they took goal waste from 52% to 5% and all four explorers in the
   last two runs finished on their own. Do not regress that.

**Measure, per map, coordinated vs independent, one seed each:** merged area,
time to 90% of final, **area covered twice** (the overlap the shared map is
supposed to eliminate), and goals wasted.

**Gate:** on at least one map, coordinated with the shared map measurably
reduces duplicated coverage against independent, *and* neither rover stalls when
the merge drops. If coordination still does not help, **report that honestly** —
it is a real result, and the reason (rendezvous arrives late) is already
documented.

---

## Phase 3 — make it distributed (target ≤ 1.5 h)

Today one central `shared_map_merger` does the work. For the lab, each rover
must do its own.

1. Per-rover merger: subscribe to the peer's `/{peer}/map`, align locally,
   publish `/{self}/shared_map` for that rover's own use.
2. No node that both rovers depend on. Kill the laptop mid-run and both must
   keep exploring on what they have.
3. Only maps, tag detections and claims cross the network. Scans, clouds and
   costmaps stay on the rover that produced them — there is a recorded finding
   (commit `d241087`) that our own DDS traffic starves the rover firmware.

**Gate:** a run where the central node is killed halfway and both rovers keep
going, with the merged maps on each rover agreeing to within the alignment error.

---

## Phase 4 — ready for the lab (target ≤ 2 h)

The real stack is still single-robot end to end. `LAB_SESSION.md` §4 lists
exactly what remains; the blocker is `real_mapping.launch.py`, which has no
`robot_ns` and owns slam_toolbox's absolute-`/map` remap plus the
field-validated velocity guard. Namespace it **behind a default that leaves
single-rover behaviour byte-for-byte unchanged**, then:

1. Bring the whole chain up twice under `rob_a` and `rob_b` with prefixed TF
   frames, in simulation, through the **real** launch path.
2. Rehearse `office_world` through that path and reach the Phase 1 gate again.
3. Update `LAB_SESSION.md`: exact bring-up order, what to type on each machine,
   the marker card, the DDS settings, what to check before trusting anything,
   what to do when it stalls.
4. Check clock synchronisation between machines — peer poses are TF lookups
   across the network and skew turns straight into position error.

**Gate:** the rehearsal run passes, and `LAB_SESSION.md` is something a person
can follow at 9am without asking questions.

---

## Verified facts — do not spend the night rediscovering these

Every one was measured on this branch. Trust them.

| Fact | Detail |
|---|---|
| `marker_length` is **0.20** | Not 0.1333, not the 0.15 default. The textures carry no quiet zone; the black square fills 100% of the image. Confirmed in sim: 1.5 cm error at 1.5 m. |
| `max_range` **4.5 m** | At 5+ m a marker is ~25 px and its map position wanders metres. A bad landmark is worse than no landmark — it is persistent and anchors the outlier gate. |
| SLAM needs the **IMU** | Raw wheel odometry (12% skid-steer yaw scale) diverged one rover's heading by ~114° and shattered its map. `robot_localization` EKF, gyro yaw *rate* only, never the ground-truth-derived quaternion. |
| `zero_origin:=true` | Without it `sim_realism_odom.py` seeds on the true world pose, both maps land in the world frame, and the transform to recover is identity by construction — a fake problem. |
| Grid matching flips | 90° or 180°, 4 runs out of 4, at confidence 0.60–0.75. This is Phase 1's whole problem. |
| One marker is not enough | Single-marker transforms on the same run ranged **0.24 m/1.2° to 14.98 m/72.3°**. Geometrically sufficient, numerically unusable, and you cannot tell which you have. |
| Markers must not be collinear | Four markers spanning 13.8 m × 3.0 m gave a 14.8° error; adding one off the line took confidence 0.35 → 0.78 and it locked. |
| Coordination only runs after lock | Before the alignment exists, `_peer_xy` returns None and "coordinated" is byte-identical to "independent". Measured active share: office 23%, depot 40–76%. |
| Tag orientation fusion | Implemented, measured, **left off**. Better offline on 4 runs, then turned a perfect 180.0° into a 7.1° error on the first fresh run. `use_tag_orientation`, default false. |
| Stale tag estimate | The aligner rejects estimates whose mean landmark residual exceeds `max_mean_error` (0.35 m) and then **keeps publishing the last one forever**. Measured landmark errors are 0.2–1.0 m, so on a drifted map it rejects nearly everything. Worth fixing. |

### Known-good numbers

Best alignment 0.12 m / 1.1° (husarion, tags). Office after the frontier fixes
0.63 m / 0.68°. Depot varies **0.19 m to 2.25 m across identical runs** — the
variance is real and is driven by per-rover SLAM drift, roughly 3 m after 19 m
of driving. Do not over-read a single good run.

---

## Ground rules — violating these wastes the night

**Launch every long run through the snapshot wrapper.**
```bash
bash scripts/run_snapshot.sh scripts/auto_multirobot_run.sh <mode> <world> <outdir> 25
```
It executes an immutable copy. **Never edit a script while it is running** —
bash reads it incrementally by byte offset and will execute garbage. This has
broken teardown twice.

**Verify a patch applied before launching anything.** A patch whose assertion
failed has already launched an unpatched 25-minute run on this project. `grep`
for the change first.

**Watch `/clock`, not `docker ps`.** `ign gazebo` segfaults inside
`/usr/lib/wsl/lib/libd3d12core.so` after 9–13 min of two-camera rendering — 4 of
4 husarion runs. The container stays up with every node running; the only
symptom is that sim time stops. The harness detects two polls with an unchanged
timestamp and salvages, but check it yourself.

**Never block for more than about two minutes in one command.** Poll, do
something useful, come back. Long sleeps have been killed mid-run and taken the
harness with them.

**Check runs while they are running.** Coverage, goal failures, common landmarks
and lock state are all visible live. If coverage is flat for two consecutive
checks while trajectories grow, the rovers are re-driving mapped space — that is
a livelock, not progress. Kill it, fix it, restart.

**Use the GPU.** Runs are sequential — two camera-on two-rover sims will not
hold real-time. Everything that is *not* a simulation runs in parallel: builds,
the benchmark, rendering, analysis, writing.

**Look at the pictures before believing a number.** Every run must produce, and
you must actually open: `merged_map.png`, `leo1_map.png`, `leo2_map.png`,
`traj_overlay.png`, `coverage.png`, `alignment.png`, plus camera frames.
`scripts/render_multirobot_media.py` and `scripts/render_marker_map.py` make
them. Doubled or offset-parallel walls, a seam, speckle in free space, geometry
outside the world, a flatlined trajectory, or both rovers in the same rooms all
mean **FAILED**, whatever the coverage says. Write what you saw, in words.

**"Weird enough to re-run"** — the only justification for a second seed: doubled
walls while the reported alignment error is under 0.5 m; a rover wedged in the
first 3 minutes; coverage going backwards; coordinated differing from
independent by more than 2×. Anything else, including "coordinated lost", gets
reported.

---

## Every tick, in this order

1. Read `reports/night_2026-08-25/PROGRESS.md`. What phase, what is running,
   since when.
2. Anything running? `docker ps` for `leo_sim`. Gone with no phase marked
   complete → capture `docker logs --tail 200 leo_sim` into the ledger.
3. Sim time advancing? Compare the last timestamp in `traj_leo1.csv` to last
   tick. Frozen with a live container is the D3D12 crash.
4. Progress? `traj_*.csv` rows, `coverage_leo*.log` last value, goal failure
   count, common landmark count, lock state.
5. Past the 25-minute cap → kill it, mark it timed-out with whatever evidence
   exists, move on. Never let one run eat the night.
6. **If nothing is running, do offline work** — the benchmark, analysis,
   rendering, the report. Never idle waiting.
7. Append a ledger line: timestamp, phase, what is running, the numbers, your
   one-line judgement.
8. A run just finished → **media review before anything else**, verdict in the
   ledger in words, then start the next thing.

---

## Deliverables — `reports/night_2026-08-25/`

- [ ] `PROGRESS.md` — the ledger, appended every tick
- [ ] `REPORT.md` — what was built, what ran, what the pictures showed, honest
      limitations, what is left. State n=1 wherever it is n=1.
- [ ] Phase 1: benchmark table before and after, and the method written down
- [ ] Phase 2: per-map coordinated vs independent, with duplicated-coverage
- [ ] Phase 3: the network-partition run
- [ ] Phase 4: `LAB_SESSION.md` updated, rehearsal run media
- [ ] Every merged map as `.pgm`/`.yaml` **and** rendered `.png`
- [ ] Dashboard rebuilt: `python3 scripts/build_multirobot_dashboard.py reports/night_2026-08-25 -o reports/night_2026-08-25/dashboard.html`
- [ ] Committed and pushed to `feat/multi-robot-integration`. **Stage your own
      files explicitly** — someone else is working in this repo and `git add -A`
      has already swept up their work once.

**If a phase cannot be reached, say so and say why.** A short true report beats a
long one. The worst outcome is a confident claim that does not survive contact
with the lab at 9am.
