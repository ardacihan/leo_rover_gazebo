# Leo Rover — Autonomous & Collaborative Exploration: Report Package

This folder contains everything you need to write up the project: plots, videos,
final-map images, raw maps, and this document explaining what each artifact is,
what was done, and what was found — honestly.

The work has two parts:

- **Part A — Single-robot autonomous exploration** (mature, fully verified). A
  hardened frontier-exploration + item-search stack for one Leo Rover.
- **Part B — Two-robot collaborative exploration** (new). Two rovers doing
  collaborative SLAM + a coordination algorithm, plus an honest benchmark.

There is also an interactive HTML version of the Part-B write-up: **`report.html`**
(open in a browser).

---

## Folder layout

```
report_package/
├── README.md                     ← this file
├── report.html                   ← interactive Part-B report (open in browser)
├── two_robot/                    ← PART B: two-robot collaborative exploration
│   ├── plots/                    ← coverage curves, map+trajectory panels, separation
│   ├── videos/                   ← time-lapse mp4s (office + large world)
│   └── final_maps/               ← final map images (map + rover trail)
├── single_robot_history/         ← PART A: single-robot exploration (PR1–PR4)
│   ├── plots/                    ← explorer comparison + verification figures
│   ├── videos/                   ← final clean runs on 3 worlds + explorer comparison
│   └── pr_reports/               ← the original PR verification write-ups (pr1–4, engine)
└── raw_maps/                     ← .pgm/.yaml occupancy maps (reusable in Nav2/RViz)
```

---

# PART A — Single-robot autonomous exploration (verified, working)

A single Leo Rover explores an unknown world end-to-end: SLAM (`slam_toolbox`) +
Nav2 + a custom `leo_rover_exploration` frontier explorer, all headless in Gazebo
under WSL/GPU. This was built and hardened over four PRs and a full verification
pass. **This part works and is verified** — it is the foundation the two-robot
work builds on.

### What each PR added (see `single_robot_history/pr_reports/`)

| PR | What it added |
|----|---------------|
| **PR1** | Hardened frontier explorer: goals snapped into known-free space and **pre-validated** with Nav2 `ComputePathToPose`; blacklist with TTL + strike escalation; goal hysteresis (10 s commit + margin); stall ladder (clear costmaps → then blacklist); watchdog for livelocks. |
| **PR2** | Nav2 controller upgrade: DWB → **RotationShim + Regulated Pure Pursuit** + PoseProgressChecker. Cleaner rotate-then-drive, faster traversal, far fewer false "failed to make progress" aborts. |
| **PR3** | Last-resort **blind escape** recovery: after repeated planner rejections while stationary, the rover briefly bypasses Nav2 with a bounded reverse/rotate to free itself, then resumes. |
| **PR4** | **Camera-coverage sweep + item search**: after frontiers are exhausted, drive viewpoint goals until the RGBD camera has observed ≥90% of walls; detect + verify ArUco items with a standoff re-observation. |

### Verification results (July 5 2026 — see `pr_reports/engine_verification.md`)

Full item-search runs, each terminated by itself, returned home, saved map:

| World | Size | Items found | Camera coverage | Map |
|-------|------|-------------|-----------------|-----|
| `office_world` (corridor + 5 offices) | 24×16 m | **8 / 8** (≤2 cm error) | 87% | clean |
| `leo_world` (multi-room arena) | 20×20 m | **6 / 6** | 90% (target) | clean |
| `depot_world` (small rooms) | 14×14 m | **4 / 4** | 90% (target) | clean |

Also validated the custom explorer against the `explore_lite` baseline (it maps
comparably and adds item search + robust recovery).

### Part-A media — what each file shows

**`single_robot_history/plots/`**
- `verification_final_maps.png` — the final saved maps for all verification runs side by side (clean, complete occupancy grids).
- `verification_coverage_curves.png` — mapped-area-over-time for the verification runs (shows steady convergence to a full map).
- `custom_vs_explorelite_coverage.png` — coverage-over-time, our custom frontier explorer vs the `explore_lite` baseline.
- `custom_vs_explorelite_maps.png` — final maps, custom vs `explore_lite`, side by side.

**`single_robot_history/videos/`** (map building + rover trail, time-lapse)
- `office_world.mp4`, `depot_world.mp4`, `leo_world.mp4` — final clean single-robot runs on the three worlds.
- `custom_frontier_explorer.mp4` vs `explore_lite_baseline.mp4` — the head-to-head explorer comparison.

**`single_robot_history/pr_reports/`** — the original detailed write-ups (`pr1.md`…`pr4.md`, `engine_verification.md`) with metrics, parameter values, and failure analyses.

---

# PART B — Two-robot collaborative exploration (new)

Two namespaced rovers (`leo1`, `leo2`), each running its **own** `slam_toolbox`
and Nav2 on its **own aligned map**, with a small deterministic **map compositor**
stitching the two maps into one shared grid. On top is a **distributed
coordinated frontier-allocation algorithm**.

### The coordination algorithm (`src/leo_rover_exploration/coordination.py`)

Every planning cycle, each rover independently runs the **same deterministic
greedy assignment** over all rovers:

- **utility = information gain − travel cost** → the better-placed rover wins each frontier (the classic Burgard-style coordinated-exploration objective);
- a **proximity discount** → once a frontier is taken, nearby frontiers lose value for the other rover, pushing the pair into disjoint regions.

No central node: peer positions come from the shared TF tree, peer commitments
from a shared claim topic. Unit-tested (6 tests, incl. the fan-apart case).

### What works ✅

**The coordination reliably divides the environment.** In every run the two
rovers split the building into separate territories instead of crowding the same
rooms — you can see it directly in the trajectory panels, and the inter-rover
separation is consistently larger when coordinated. That is the algorithm doing
exactly what it's designed to do.

### The honest benchmark result ⚠️ (this is important)

Despite the coordination working, **a clean "two robots explore faster than one"
was NOT established** in these simulations. It is **not** that two robots are
worse — the comparison is **inconclusive**:

1. **Run-to-run variance is ~±30%.** The same coordinated condition reached
   505 m² at t=330 s in one run and 386 m² in another. The single-vs-two-robot
   difference is *inside* that noise — resolving it needs 3–5 runs per condition,
   averaged, which was not done.
2. **Two-robot compute interacts with sim speed.** Doubling SLAM + Nav2 degrades
   control fidelity in a way that depends on the real-time factor, confounding a
   sim-time comparison.
3. **A 20 m lidar makes one robot very efficient** in open rooms (it maps across
   a room without entering), leaving little for a second robot to add.
4. **Fixed overheads** (startup clustering + one rover lingering to finish the
   last rooms while the other idles) are a large fraction of a short run.

> **Correction:** an earlier draft claimed "+87% from coordination." That was
> **wrong** — an artifact of cutting the uncoordinated runs off at a time cap
> before they finished. Run to completion, the uncoordinated pair catches up.
> This package reflects the corrected finding.

The same pattern held on a purpose-built large world (30×24 m, nine rooms): the
rovers divided it cleanly, but two coordinated rovers reached full coverage at
t≈1000 s vs t≈810 s for a single robot — not the 2× a bigger map was meant to
reveal.

### Part-B media — what each file shows

**`two_robot/plots/`**
- `office_coverage_vs_time.png` — mapped area over time for 1 robot / 2 uncoordinated / 2 coordinated (office_world), all measured identically and clipped to the true world extent. **The three curves sit essentially on top of each other** — this is the honest headline.
- `office_maps_and_trajectories.png` — final maps with rover paths. Left: single. Centre: uncoordinated (paths overlap in the middle). Right: coordinated (leo1 red / leo2 purple split into separate regions). **This shows the coordination working.**
- `office_rover_separation.png` — distance between the two rovers over time; coordinated holds them farther apart than uncoordinated.
- `depot_*` — the same three plots for the smaller, more open `depot_world`.

**`two_robot/videos/`** (time-lapse of the merged map building)
- `office_2robot_coordinated.mp4` / `office_2robot_uncoordinated.mp4` / `office_1robot_baseline.mp4` — the three office conditions.
- `bigworld_2robot_coordinated.mp4` / `bigworld_1robot_baseline.mp4` — the large 9-room world.

**`two_robot/final_maps/`** — final map + trajectory images: `office_{coordinated,uncoordinated,single}_map.png`, `bigworld_coordinated_map.png`.

**`raw_maps/`** — the raw `.pgm` + `.yaml` occupancy maps (load in RViz or as a Nav2 map): `office_2robot_merged.*`, `bigworld_2robot_merged.*`.

---

## Engineering fixes made along the way (all real, worth keeping)

These were non-obvious infrastructure bugs, each of which silently broke the
two-robot system:

- **`slam_toolbox` `/map` clobbering** — slam publishes to the *absolute*
  `/map`, ignoring the namespace, so both rovers fought over one topic and the
  per-rover maps had no publisher at all. This was the root cause of the second
  rover getting stuck. Fixed by remapping `/map → /leo{i}/map` per rover.
- **The GPU was idle** — the headless Gazebo server was rendering every lidar and
  camera in *software* (Mesa `llvmpipe`) because it lacked the WSL GPU libraries
  on its library path. Moving rendering onto the GPU (RTX 4060 Ti) took the raw
  sim from 0.94× to 1.73× real-time. *(Check `nvidia-smi` mid-run to confirm.)*
- **Merged-map seams blocked doorways** — planning on the stitched map raised
  spurious walls where sub-maps overlapped. Fixed by having each rover plan on
  its *own* aligned map; the merged map is only for coordination and the metric.
- **Coverage metric over-counted** from odometry drift → fixed by clipping to the
  world extent so all conditions are measured on the same footprint.
- **Watchdog kills** — uncapping the sim rate spiked the CPU and tripped a
  watchdog that killed long runs → fixed with a stable real-time-factor cap.

---

## Bottom line

- **Part A (single-robot exploration + item search): done and verified.** Use the
  verification maps/curves/videos freely — those results are solid.
- **Part B (two-robot collaborative): the system and algorithm work** (rovers
  build a shared map and reliably divide the space), **but a net exploration-
  speed advantage over one robot is not established** in these sims and would
  require a small averaged multi-run campaign to settle. Everything is scripted
  (`auto_collab_run.sh`, `auto_explore_run.sh`, `analyze_collab.py`,
  `make_big_world.py`) to run that campaign later if desired.

Write it up honestly: the collaboration behaviour is demonstrable and the
engineering is real; the quantitative speed claim is not yet supported.
