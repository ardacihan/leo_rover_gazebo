# Overnight Goal — Report-Ready Collaborative Maps + Item-Search Benchmark

**Session type:** autonomous overnight run. Work top-down by priority; a finished
Priority 1 beats half-finished everything. **Quality over run count.**

## Mission

Produce **report-ready two-robot collaborative SLAM results**: clean, noise-free,
perfectly aligned merged maps, and an **item-search (ArUco) benchmark** that
replaces raw map coverage as the headline metric. Implement/fix whatever
algorithm work is needed to make the collaborative run and landmark detection
demonstrably better — that is the aim, not accumulating sim hours.

---

## Priority 1 — Robust, aligned, report-ready merged maps

The current merged maps (`reports/collab_final/figures/maps_*.png`) have gray
speckle fields, doubled/fuzzy walls, and visible seams. Fix the pipeline, not
just the picture:

1. **Noise removal in the compositor** (`scripts/map_compositor.py`):
   probabilistic fusion instead of naive overwrite (log-odds or
   max-confidence per cell), plus post-filtering — remove isolated occupied
   cells (connected-component / median filter), clip to world bounds
   (`map_coverage.py` already supports bound clipping — reuse that logic).
2. **Alignment correction.** Fixed spawn offsets (`OFFSETS` in
   map_compositor.py) ignore per-robot SLAM drift, which is what doubles the
   walls. Refine the leo2→leo1 transform with occupancy-grid registration
   (ICP on occupied cells, or correlative scan matching) **seeded by the known
   offsets** — small search window, so it's fast and can't diverge.
3. **Validation metric:** score each merged map against the world SDF ground
   truth (wall IoU / alignment RMSE). Also report the registration correction
   vs the ground-truth offsets — with known spawns the alignment error is
   directly measurable. A merged map is "report-ready" when it is visually
   indistinguishable from a single-robot map: single-pixel walls, no speckle,
   no seam band.

**Acceptance:** office_world + depot_world merged maps regenerated (from
existing saved per-robot maps in `maps/` first — no sim needed to iterate on
fusion!), before/after figure, wall-IoU numbers in a table.

## Priority 2 — Item-search benchmark (replaces coverage)

Port the PR4 single-robot item search (SWEEPING + VERIFY states,
`camera_coverage.py`, `mock_aruco_detector.py`, item registry in
`frontier_explorer.py`) into the two-robot stack:

1. **Shared item registry:** found-item claims broadcast between robots
   (extend `/exploration_claims` or a new `/item_claims` topic), dedup by
   position in the common frame (`_get_common_offset` already exists in
   `coordination.py`).
2. **Shared camera-coverage claims:** a robot must not re-sweep walls a peer
   has already observed (exchange observed-cell sets or sweep-target claims).
3. **Cross-robot verification (if time permits):** nearest peer does the
   confirming second observation instead of the discoverer driving a standoff
   viewpoint; measure detour distance saved.
4. **Benchmark matrix:** office_world, 6–8 markers, **2 seeds only** (two
   spawn/marker layouts), conditions: 1-robot, 2-robot independent, 2-robot
   coordinated. Metric: **time-to-all-items** and time-to-k-items (sim time).
   Cameras must be ON (`enable_camera` xacro arg).

**Acceptance:** `reports/item_search_collab/` with per-run logs, a
time-to-items comparison figure, found-items overlay on the clean merged map,
and a short honest summary.md (state variance limits explicitly — n=2).

## Priority 3 — Relax the known-spawn assumption (keep it simple)

Reuse the Priority-1 registration, but seeded from a **coarse prior instead of
exact offsets** (e.g., assume spawn error up to ±1 m, ±15°). Show the
registration recovers the true offset; report recovered-vs-true error. It is
fine to assume overlapping initial coverage and rough odometry priors — say so
in the report. Full unknown-pose rendezvous is OUT of scope tonight.

---

## Hardware & efficiency requirements (hard constraints)

Machine: RTX 4060 Ti 16 GB, i7-14700 (28 threads). **Use them.**

- **Verify the GPU is actually engaged** in the first 2 minutes of every sim
  batch: `nvidia-smi` must show the ign/gz process. The WSL headless server
  silently falls back to llvmpipe (CPU) without `/usr/lib/wsl/lib` on
  `LD_LIBRARY_PATH` — `two_robots_gpu.launch.py` already injects `gpu_env`
  into gz_server; make sure any new launch path does the same, especially now
  that cameras are ON (RGBD rendering is the GPU load).
- **Parallelize non-sim work always** (builds, map post-processing, analysis
  across runs, figure generation).
- **Parallel sims:** the two seeds of a condition MAY run concurrently in
  separate containers *only if* a probe shows both hold real_time_factor
  ≥ ~0.9 with cameras on. Measure once at the start; if RTF degrades, run
  sequentially — control fidelity couples to RTF and corrupts results.
  Keep RTF capped at 1.0 (uncapped spikes CPU and gets watchdog-killed).
- **Time caps:** hard cap **20 min wall-clock per sim run** (pass timeout to
  `auto_collab_run.sh`). Full experiment matrix ≤ ~3 h wall total. Iterate map
  fusion offline on saved maps (seconds per iteration), never via re-simming.
- **Smoke test before batch:** one 3-minute run per new launch config —
  check `ros2 topic info /leo1/map -v` (publisher count must be ≥1; 0 means
  the slam `/map` remap regressed), check detections arrive, check GPU — then
  launch the batch in background with logs.

## Known landmines (do not rediscover these)

- slam_toolbox publishes to absolute `/map`; the per-robot remap in
  `slam_multi.launch.py` is load-bearing. Never use multirobot_map_merge's
  known-init-pose params (silently broken on Humble) — the compositor is the
  merge path.
- Navigation must stay **own-frame** (each Nav2 on its own `/leo{i}/map`);
  the merged map is for metrics/report only. Seam artifacts on a shared nav
  map block doorways.
- Coverage from the compositor over-counts under drift — clip to world bounds.
- husarion_office wedges rovers with cameras off; use office_world/depot_world.
- Docker runs from PowerShell (not bash) with `--entrypoint bash`; headless
  verify workflow per project scripts; `pkill` self-match caveat.

## Out of scope tonight

- 3 robots. Multi-hour soak runs. Real (non-mock) ArUco perception.
- Full unknown-pose map merging / rendezvous SLAM.
- Re-litigating the coverage-speedup question (the null result stands; the
  new metric is item search).

## Deliverables checklist

- [ ] Clean merged maps (office + depot), before/after figure, wall-IoU table
- [ ] Registration module + recovered-vs-true offset numbers (exact & coarse prior)
- [ ] Multi-robot item search implemented (shared registry + coverage claims)
- [ ] Benchmark: 3 conditions × 2 seeds, time-to-items figure + summary.md
- [ ] All figures publication-quality and assembled under `reports/overnight_<date>/`
- [ ] Honest limitations section (n=2, mock detector, coarse-prior assumption)
