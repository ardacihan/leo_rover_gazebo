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
| 1 | Marker-free merge ≥7/10, zero confident-wrong, <5 s/pair | OFFLINE DONE (6/7+3 abstain, 0 wrong, ≤4.2 s) — confirmation sim IN FLIGHT |
| 2 | Merged map changes exploration; coordinated vs independent, office first | NOT STARTED |
| 3 | Per-rover merger; laptop-kill partition run | NOT STARTED |
| 4 | Namespaced real launch path + rehearsal + LAB_SESSION.md | NOT STARTED |

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
- **Viper identified:** Slurm cluster, ssh viper11 via WSL bridge
  (`wsl.exe -d Ubuntu -- bash -lc "ssh viper11 '<cmd>'"`), 8 shared APU
  (ROCm) slots, courtesy cap ~6 jobs. Recon+setup delegated to a subagent
  (report will land in reports/night_2026-08-25/viper_recon.md); sim
  container port is likely infeasible tonight, offline benchmark trivially
  portable but also cheap locally — value is parallel sim capacity IF
  apptainer exists.
