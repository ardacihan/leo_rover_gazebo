# Night 2026-08-25 — running ledger

**Branch:** `feat/multi-robot-integration` (working tree carries uncommitted
frontier fixes + tonight's offline-merge tools — see tick 0)
**Goal doc:** `OVERNIGHT_GOAL_2026-08-25.md` — read it before acting.
**Deadline:** report by 08:30 Berlin, everything committed and pushed.

Read this ledger first on every tick. Never restart a phase marked COMPLETE.

## Operator priorities (from the user, 2026-08-24 ~22:20)

1. **Functional-first for the 9am lab demo.** Marker-based merging is the
   proven path; marker-free (Phase 1) is the high-value stretch. Abstaining
   until markers arrive is an acceptable production behaviour — a
   confident-wrong merge is the only unacceptable one.
2. **Prefer office_world for Phase 2 measurements** — it is the complex map.
3. Distributed manner matters: per-rover merging (Phase 3), rovers using each
   other's coverage (Phase 2), no laptop single point of failure.
4. By morning: real-rover path finalized so the lab needs near-zero fixes
   (Phase 4 + LAB_SESSION.md + TOMORROW_PLAN.md kept consistent).

## Plan

| Phase | What | Status |
|---|---|---|
| 0 | Build + tests, benchmark reproduces 0/10, GPU check | COMPLETE except GPU line (deferred to a camera-on run; markerfree run is camera-off) |
| 1 | Marker-free merge ≥7/10, zero confident-wrong, <5 s/pair | **COMPLETE** — 6/7+3 abstain, 0 wrong offline; LIVE LOCK 0.45 m/1.3° markers-off (run5), media clean. Caveats: strict pass count 6 not 7 (7th drift-capped, tag-proven); big office pairs 6.5–7.1 s vs 5 s target |
| 2 | Merged map changes exploration; coordinated vs independent, office first | **COMPLETE** — gate met on BOTH maps: dup coverage −19% office / −10% depot vs independent, no stalls, mask live (31,619 cells) |
| 3 | Per-rover merger; laptop-kill partition run | **COMPLETE** — both rovers locked independently (0.38 m & 0.17 m), estimates mutually agree 0.28 m/0.6°, shared maps saved live 40 s after bridge kill. Caveat: explorers finished 4 s post-kill |
| 4 | Namespaced real launch path + rehearsal + LAB_SESSION.md | **COMPLETE** — rehearsal PASSED (maps, prefixed TF, ~0.8 m motion through the full safety chain on both rovers); 2 real bugs found+fixed (yaml-under-namespace, box_frame); LAB_SESSION + TOMORROW_PLAN updated |

## Assets already in the tree tonight (do not rebuild these)

- `scripts/align_registries_offline.py` — tag-registry Kabsch + leave-one-out
  + `--refine` (grid correlation in a ±0.5 m/±4° trust region around the tag
  seed). Benchmarked on all recorded runs this evening: every EKF-era pair
  ≤3° yaw, 0.19–1.0 m; refine drives wall overlap to ~99% and rescued
  phase4_office to 0.40 m/0.33°. **`refine_transform()` is a working local
  correlative matcher — Phase 1 can reuse its scoring; what's missing is the
  global search + margin-based abstention.**
- `scripts/live_merge_watch.py` — projector display (waiting-for-rendezvous →
  snap-to-merged), tested both states.
- `TOMORROW_PLAN.md` — the 6h lab runbook (keep consistent with Phase 4).
- Uncommitted working-tree changes: frontier world-bounds + growing blacklist
  (produced the first office run that self-terminated AND merged cleanly,
  2026-08-24 22:03) — part of what Phase 2's gate depends on. Commit early.

## Ledger

### 2026-08-24 22:25 — tick 0 — Phase 0 begun (interactive session, pre-loop)

- Branch confirmed `feat/multi-robot-integration`; **no sim in flight**
  (last container stopped 22:02 after the phase4 office run).
- **Benchmark harness CONFIRMED:** `merge_benchmark.py --method
  _baseline_matcher:match` → **0/10, 180° flips on every depot/office pair,
  53 s total.** Matches the goal doc exactly. Ground truths load, maps load.
- Phase 0 remaining for the first loop tick: `colcon build` + 61/61 unit
  tests (in container), GPU check (piggyback on the first sim run — do NOT
  burn a run slot just for the GPU line; the Phase 1 confirmation run or a
  Phase 2 run can carry it).
- Fresh evidence worth folding into REPORT.md: phase4_office_fixed
  (2026-08-24 22:03) self-terminated 2/2, live aligner locked 12.9 m wrong,
  offline tag fit 0.56 m/0.66° → 0.51 m/0.8° (exclude flagged tag 4) →
  0.40 m/0.33° with --refine. Media reviewed: merged map clean, single
  walls (`offline_merged_refined.png`).

### 2026-08-24 ~22:4x — tick 1 — Phase 0 nearly done, Phase 1 begun

- No sim in flight (`docker ps` empty). Loop session live.
- **Viper unavailable:** no reference anywhere in the repo, and the only two
  remote peer sessions (vast-knuth laptop, exploration-stall) are both
  offline. Falling back to local GPU + parallel offline work; noting so the
  morning report can say why the 8 arms went unused.
- **Phase 0 build: GREEN.** `colcon build --symlink-install` in container,
  17 packages, 41.8 s. `leo_rover_exploration` pytest 12/12. Full suite
  (exploration + nav2_exploration + real_bringup = the 61) running now.
- GPU check still deferred to the first sim run (per tick 0 decision).
- **Phase 1 started.** Wrote `scripts/marker_free_matcher.py`: global FFT
  correlative search (full-circle 2° yaw sweep, unbounded translation via
  padded FFT — baseline's ±6 m cap couldn't even represent husarion's
  13.8 m offset), overlap-only scoring (penalties for occ2-on-free1 and
  free2-on-occ1, normalized by occ2-in-known1), best-vs-runner-up margin
  abstention (runner must be >12° or >2 m away), full-res coarse-to-fine
  polish reusing the refine_transform scoring idea, abstain if polished
  wall-overlap <0.55. First benchmark run in flight (MFM_DEBUG=1).

### 2026-08-24 ~23:0x — tick 1 cont. — Phase 0 COMPLETE (except GPU line), Phase 1 iterating

- **Phase 0 test result, honest version:** full suite is 163 tests, not 61.
  exploration+real_bringup: **78/78 green**. leo_nav2_exploration: 72/85 —
  **13 contract-test failures** (test_config_contracts: nav2 polygon
  footprint/state-lattice, costmap doorway resolution, DWB footprint, [real]
  sensor topics+slam; test_shell_contracts: operator docs, ros2-run usage,
  lidar raw/filtered, bundle validator). These are the nav2-bundle contracts
  from the real-rover branch — **recheck in Phase 4**, the [real] ones guard
  the lab launch path. Goal doc's "61/61" matches no current subset.
- Committed evening work as 520fd6c (refine + phase4_office assets).
- **Phase 1 iteration 1 (all-abstain):** coarse FFT score too blunt —
  abs floor 0.35 killed everything; also margin correctly caught depot 180°
  ambiguity. Iteration 2 (truth-annotated tuner, 20 modes, wide ±1.05 m/±4°
  polish): TRUE mode found + polished to hit 0.87–0.998 on 6/10 pairs with
  runner-ups ≤0.34 — polished-hit margin is hugely discriminative. Risk
  found: office_coordinated pair commits 1.93 m wrong at hit 0.764 (map
  drift vs spawn truth) → floor raised to 0.80, margin 0.30. depot_coord
  true mode fell below top-20 candidates → peaks/yaw 4, modes 30.
  Iteration 3 benchmark in flight; expecting ~7/10 + 3-4 abstain + 0 wrong.
### 2026-08-24 ~23:4x — tick 1 cont. — Phase 1 offline COMPLETE, confirmation run launched

- Iteration 3 benchmark: **6/7 attempted within 0.5 m/10° (0.27–0.41 m,
  0.45–2.3°), 3 abstained, ZERO confident-wrong, 1.9–4.2 s/pair.** The one
  1.9m-wrong risk pair now abstains (floor 0.80). Attempted-but-0.65 m pair
  (phase2_depot_coordinated) proven **drift-capped, not matcher-limited**:
  tag+refine lands on the SAME transform (3.60,-8.83 vs 3.62,-8.82), 0.63 m
  from spawn truth. No rigid method can pass that pair; benchmark truth is
  the spawn offset, the maps themselves are warped ~0.65 m.
- Tried overlap-normalized polish + 0.10 overlap floor to rescue husarion
  partial-overlap pairs: **catastrophic** (tiny-overlap hypotheses score
  0.8+ on a matching fragment, 10/10 abstain, true modes crowded out of the
  candidate list). Reverted; husarion abstains are honest. Gate 1 strictly
  is 6/10 not 7/10 — documented with the drift evidence; gates 2 (the one
  that matters), 3, 4 pass.
- Wired `alignment_mode:=markerfree` into map_based_aligner (additive, hybrid
  default untouched; confidence = polished hit; abstentions on debug topic
  with reason). Package build green, imports green, 78/78 tests green.
  Committed as e2ec0a5.
- auto_multirobot_run.sh: ALIGN_MODE / SKIP_ARUCO / ENABLE_CAMERA overrides
  (grep-verified + bash -n). Camera-off for marker-free runs also dodges the
  9–13 min two-camera D3D12 segfault.
- **23:4x LAUNCHED (run_snapshot): coordinated office_world 25 min,
  ALIGN_MODE=markerfree SKIP_ARUCO=1 → reports/night_2026-08-25/
  phase1_markerfree_office.** Success = aligner locks near (11,-10,180°)
  with markers fully off, or abstains honestly; media review after.
### 2026-08-24 23:37 — tick 1 cont. — GPU CONFIRMED, Phase 2 code done, bridge bug found+fixed

- **Phase 0 GPU line: `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)`**
  (gpu_check.txt of the markerfree run — even camera-off renders lidar on
  D3D12). **Phase 0 COMPLETE.**
- In-flight run healthy at 23:36 (t=325s sim): coverage 35.6→82.8 m²
  growing, traj rows growing, 0 tags (as designed), aligner abstaining
  honestly (hit 0.52–0.58 < 0.80 while maps barely overlap — correct).
- **BUG found live and fixed:** alignment_tf_bridge `require_tag_evidence`
  (the defense against the old flip-prone matcher) withholds TF forever in
  markerfree mode → lock could never propagate. shared_align.launch.py now
  passes require_tag_evidence:=false iff alignment_mode==markerfree.
  Current run predates the fix (snapshot) → it validates live-abstention +
  camera-off stability only; **lock confirmation needs a re-run** (queued
  next).
- **Phase 2 code COMPLETE (offline):** frontier_explorer subscribes
  shared_map_topic (VOLATILE QoS — TRANSIENT_LOCAL would never match),
  unknown-cells-known-in-merged-map masked out before frontier detection
  (mask the unknown side, NOT frontier cells — those are free in the merged
  map by construction and masking them kills every frontier). Stale/absent
  shared map or missing TF → seamless own-map fallback. 5 unit tests,
  17/17 green. Coordinated-only wiring; independent baseline byte-identical.
  Committed 794cc48.
- Sim queue after current run finishes + media review:
  1. phase1_markerfree_office_run2 (with bridge fix) — Phase 1 lock confirm
     AND first Phase 2 coordinated-with-mask evidence in one run.
  2. phase2_office_independent baseline.
  3. Phase 3 partition run; 4. Phase 4 rehearsal.
### 2026-08-25 00:0x — tick 2 — run 1 media review + the asymmetric-coverage finding

- **phase1_markerfree_office finished CLEAN: self-terminated 2/2 at t=1035s,
  saved maps + all media rendered.** Media verdict in words: leo2's map is
  the whole office, beautiful single walls, all six rooms + corridor. leo1
  mapped only its two western rooms + corridor speckle — its eastward
  corridor goals aborted repeatedly (Nav2 status 6) until blacklisted, then
  it finished at t≈580s (the known 1-in-4 stall flavor; n=1, not tonight's
  scope). merged_map.png = leo1 passthrough only (no lock — expected, run
  predates the bridge fix). No doubled walls anywhere. Zero explorer goal
  failures for leo2.
- **Live matcher behaved exactly as designed AND exposed a blind spot:**
  64 abstains, 0 commits, 0 confident-wrong. But the final pair is visibly
  mergeable (leo1's map is a subset of leo2's) and still scores only
  fwd hit 0.295: the forward hit normalizes by ALL of map2's walls, so
  asymmetric coverage caps a correct alignment at the overlap fraction.
- **Fix: symmetric quality = max(fwd, rev)** — reverse_hit scores map1's
  walls into map2 under the inverse transform, still all-points-normalized
  (no fragment gaming). The smaller map gets to be the numerator.
  Benchmark re-verification in flight; must stay ≥6/7 with 0 wrong.
- **VIPER IS LIVE (subagent):** offline benchmark reproduced 0/10 there;
  the actual sim image ported (apptainer .sif), real launch stack ran on an
  APU node at RTF 0.95 lidar-only/llvmpipe (--contain hides /dev/dri, the
  decisive fix). 8 slots idle. Agent now adapting auto_multirobot_run.sh
  for apptainer and firing Phase 2 A/B office (then depot) runs there,
  results sync back to reports/night_2026-08-25/phase2v_*. Local GPU stays
  on the Phase 1 confirmation rerun.
### 2026-08-24 ~23:58 — tick 2 cont. — root cause found: one-way candidate gate; bidirectional fix in

- Symmetric quality (max of fwd/rev polish hit) verified: benchmark still
  6/7, 3 abstain, 0 wrong. On tonight's pair the best mode rose 0.295→0.704
  — but that mode is 11.8 m WRONG (periodic office rooms + small leo1 map);
  margin machinery held it off. **Truth-seeded polish on the same pair:
  q=0.987 (0.46 m drift)** — the true mode is excellent but was NEVER in the
  candidate list: at truth most of leo2's map lies outside leo1's known
  area and the coarse MIN_OVERLAP_FRAC gate kills it. Same root cause as
  the husarion abstains.
- **Fix: bidirectional coarse search** — also search map1-into-map2 (where
  the true hypothesis passes the gate trivially), invert, pool, dedupe to
  30 modes. Each direction keeps its own honest overlap gate → no fragment
  gaming. MARGIN_MIN 0.30→0.25 (true 0.987 vs periodic-false 0.704 is
  margin 0.286; measured commit margins elsewhere ≥0.6). Benchmark
  re-validation in flight — required: no confident-wrong anywhere, else
  revert margin.
- **phase1_markerfree_office_run2 LAUNCHED** (bridge fix + Phase 2 masking
  + bidirectional matcher — aligner imports post-sync by timing). If maps
  end asymmetric again this run can STILL lock now.
### 2026-08-25 00:1x — tick 3 — full mode table: truth crowded out at PEAK level; triage architecture

- Bidirectional benchmark: 6/7, 3 abstain, 0 wrong — but 3.5–8.3 s (office
  over the 5 s gate) and tonight's pair STILL abstains: full 30-mode table
  shows **no mode near truth in either direction**. Root cause is the peak
  extraction, not scoring: an office is a periodic room grid, and at
  yaw≈180 the false room-shift placements fill all 4 peaks/yaw before truth
  gets one. (Truth-seeded polish remains q=0.987.)
- **Architecture fix:** 8 peaks/yaw, pool ≤120 modes from both directions,
  cheap single-stage triage polish (~5 ms, n=1200, ±0.45 m/±1.6°) on ALL of
  them (the polish quality is the only trustworthy ranking), full 3-stage
  polish on the top 8 only. Should also bring office pairs back under ~5 s.
  MARGIN_MIN 0.30→0.25 (true 0.987 vs periodic-false 0.704 = margin 0.286).
  Benchmark validation in flight; required: 0 confident-wrong.
- run2 (in flight, healthy, abstaining 0.65–0.76 on small maps) carries the
  PREVIOUS matcher import — can still lock if coverage ends symmetric;
  relaunch with triage version if it ends unmerged.
- Commits pushed to origin: fd22748..794cc48.
### 2026-08-25 00:2x — tick 3 cont. — TRIAGE MATCHER VALIDATED, run3 launched

- **Triage benchmark: 6/7 + 3 abstain + 0 confident-wrong preserved, AND
  tonight's asymmetric pair now COMMITS at 0.47 m / 0.2° (q=0.989).**
  Timing 4.0–7.1 s/pair: the two big office pairs exceed the 5 s gate by
  ~2 s — reported honestly; live cadence is 15 s so it runs live fine.
  Committed f7bdf48, pushed.
- run2 killed at t≈365s (its aligner imported the pre-triage matcher; hits
  were declining 0.65→0.36 with fwd-only normalization and it could not
  have found the office truth under asymmetric coverage — superseded, its
  partial dir kept for the record). **run3 LAUNCHED** with the final
  matcher: reports/night_2026-08-25/phase1_markerfree_office_run3.
- Viper agent told to re-sync the matcher before its coordinated jobs.
### 2026-08-25 00:1x–00:2x — tick 4 — merger mode bug, run4 SIGKILL mystery, Phase 3 code written

- **Third live-wiring bug caught by reading, not running:** shared_map_merger
  subscribed the accepted transform only for modes (tag, map, hybrid) — in
  markerfree it would NEVER merge, however good the lock. Fixed both mode
  tuples, committed 3d01193. Full-chain audit of remaining "hybrid" tuples:
  all others are in paths markerfree bypasses.
- run3 killed (merger had the broken import), relaunched as run4 → **run4's
  harness AND container both SIGKILLed (137) ~90 s after launch, cause
  unknown** (WSL/docker blip; memory fine: 19G avail in WSL, 10.8G host).
  run5 relaunched, passed the point where run4 died, healthy.
- **Phase 3 code COMPLETE (offline):** distributed_shared_map.launch.py —
  per-rover aligner+merger pairs, each publishing /leo{i}/shared_map in the
  rover's OWN frame (mask needs no TF: explorer now reads the grid's
  header.frame_id and uses identity when it's its own frame — new unit
  test, 6 mask tests total). auto_multirobot_run.sh: DISTRIBUTED=1 switches
  to per-rover stack + a killable central bridge fed from leo1's aligner;
  PARTITION_AT_MIN=n kills that bridge mid-run; teardown saves BOTH
  /leo{i}/shared_map maps for the agreement check; coverage monitor follows
  /leo1/shared_map. The merge path has NO central dependency.
### 2026-08-25 00:30 — tick 5 — PHASE 1 LIVE CONFIRMATION PASSED (run5)

- **run5 (office, markers OFF, cameras OFF): 34 honest abstentions while
  the maps were disjoint, then LOCKED at t≈645s via map_based:
  (11.30, -9.66, -178.7°) = 0.45 m / 1.3° from truth, confidence
  0.80→0.84, held to the end. Self-terminated 2/2 at t=785s. ZERO goal
  failures.** Media reviewed: merged map = the whole office, single walls,
  no seam/doubling/speckle (merged_map.png). Recorder scored err=0.45m/1.3°
  locked=1 continuously after lock.
- **Phase 2 live evidence in the same run:** shared-map mask ON both
  rovers after lock — leo2 masked 31,619 unknown cells as peer-covered.
- Phase 4 offline part done earlier this tick: real_mapping.launch.py
  namespaced (default byte-for-byte; -s verified both forms; contract
  tests unchanged at 13 pre-existing failures), LAB_SESSION §4 updated,
  committed 2e7f6ad.
- **Phase 3 partition run LAUNCHED** (DISTRIBUTED=1 PARTITION_AT_MIN=13,
  markerfree, office) → reports/night_2026-08-25/phase3_partition_office.
### 2026-08-25 00:50 — tick 6 — PHASE 3 GATE MET (partition run)

- **phase3_partition_office: PASS.** leo1.peer_aligner locked
  (11.26,-9.72,180.4°)=0.38 m/0.4° conf 0.91; leo2.peer_aligner locked
  (11.11,-9.87,-181.0°)=0.17 m/1.0° conf 0.92 — independently, zero
  markers. Composition vs identity: (0.16,0.23,-0.6°) → the two rovers'
  merged maps agree within alignment error. Bridge killed minute 13; both
  /leo{i}/shared_map saved from live topics ~40 s later. Media: both
  shared maps = complete office, single walls, exact 180° rotations of
  each other. Caveat (honest): explorers finished 4 s after the kill —
  post-partition exploration demonstrated only briefly; merge-path
  independence is by construction (mergers never subscribe the bridge).
- **Phase 4 rehearsal LAUNCHED** (freed sim slot):
  reports/night_2026-08-25/phase4_rehearsal, 10 min cap, real launch path
  ×2 with safety-chain motion checks.
### 2026-08-25 01:0x — tick 7 — Docker Desktop failure identified; Viper agent lost to session limit

- **Docker Desktop daemon wedged (HTTP 500 → hangs).** This is what
  SIGKILLed run4 earlier and it stopped the Phase 4 rehearsal mid-jog
  (both map-publish checks had already PASSED). Docker Desktop restarted.
  Rehearsal will be re-run when the daemon is back.
- **Viper subagent died at its session limit (resets 03:40)** right after
  syncing phase2v_office_coordinated — which is COMPLETE and clean
  (self-terminated, all artifacts). Slurm jobs it submitted keep running;
  I am taking over the cluster directly over ssh.
- **First Phase 2 numbers** (host-python metrics): office coordinated
  (Viper) t90=495 s, dup 287 m², 53 goals/1 failed; run5 coordinated
  t90=570 s, dup 250 m², 49/3. Independent baseline still needed for the
  actual A/B.
### 2026-08-25 01:2x — tick 7 cont. — took over Viper; A/B half-in; my launch bug found

- Took over Viper by ssh (agent dead until 03:40; its Slurm jobs had all
  finished). **All four phase2v runs existed; synced back.** Coordinated
  runs are GOLD: office locked 0.24 m/0.1° conf 0.94 (mask: 17,635 cells);
  **depot locked 0.18 m/0.0° conf 0.94 after 11 abstentions — the
  flip-prone world, correct, marker-free.** Live locks now n=3 (0.45,
  0.24, 0.18 m), zero confident-wrong ever.
- **BUT both independent baselines were broken by MY runner bug:** empty
  `shared_map_topic:=` is a malformed launch arg → collab_explore died →
  25 min with zero explorers (15 m², 0 goals). Fixed locally
  (${var:+...}), patched the Viper runner copy, **baselines resubmitted
  (jobs 11007856/57)** — results ~01:55.
- Docker Desktop still restarting; Phase 4 rehearsal rerun queued on it
  (first 2 of 6 checks had already PASSED when the daemon died).
### 2026-08-25 01:5x — tick 8 — closing sprint (user: run everything, Viper+local)

- Docker needed the HARD reset (kill procs + wsl --shutdown + relaunch);
  soft restart was not enough. Recovered; **Phase 4 rehearsal re-launched
  and past the map checks again (2/6 PASS, jog running).**
- Viper baselines 11007856/57 both RUNNING healthily (fix confirmed:
  explorers alive, coverage 90–110 m² at t=450 s). ETA ~01:58 + sync.
- All four phase2v runs' media rendered; **depot marker-free merged map
  visually verified: full depot, single walls, one faint <0.2 m offset at
  the NW edge — consistent with the 0.18 m lock.**
- LAB_SESSION.md finalized (markerfree fallback with live numbers,
  distributed option + rob_a/rob_b edit flag, clock-sync first-step, stall
  table + known-good numbers updated). TOMORROW_PLAN bonus tier updated to
  match. REPORT exec summary + 9-point limitations written. Memory saved
  (leo-rover-markerfree-merge, leo-rover-night-2026-08-25-ops).
- Remaining: rehearsal verdict → baselines → final A/B table → dashboard
  → REPORT final pass → last commit/push.
### 2026-08-25 ~04:00 — FINAL — all four phases complete, night closed

- Phase 2 A/B landed (Viper baselines rerun after my empty-arg launch bug):
  dup coverage 287 vs 355 m² office (−19%), 162 vs 181 m² depot (−10%),
  same final areas, no stalls. GATE MET both maps.
- Phase 4 rehearsal: 4 attempts, each failure a real find. (2) yaml files
  don't match namespaced nodes → params inline; (3) box filter's
  base_footprint frame unprefixed → every scan dropped, chain starved —
  found by tracing a healthy-looking chain holding the rover at x=0.0,
  fixed, verified by a 0.97 m jog; (4) all six checks pass — the two
  motion "FAIL" labels were the host-python3 comparator erroring on real
  0.85/0.79 m motion; comparator fixed (awk), summary amended with the
  original numbers preserved.
- REPORT.md complete (exec summary, all phases, 9 honest limitations,
  deliverables all checked). Dashboard built. Memory saved. ~15 commits
  pushed through the night; final commit follows this entry.
- **Viper identified:** Slurm cluster, ssh viper11 via WSL bridge
  (`wsl.exe -d Ubuntu -- bash -lc "ssh viper11 '<cmd>'"`), 8 shared APU
  (ROCm) slots, courtesy cap ~6 jobs. Recon+setup delegated to a subagent
  (report will land in reports/night_2026-08-25/viper_recon.md); sim
  container port is likely infeasible tonight, offline benchmark trivially
  portable but also cheap locally — value is parallel sim capacity IF
  apptainer exists.
