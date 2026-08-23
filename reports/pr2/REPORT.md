# PR2 — Nav2 controller upgrade: verification report

**Date:** 2026-06-11/12 · **World:** `husarion_office` ·
**Change under test:** DWB → RotationShimController + RegulatedPurePursuit,
SimpleProgressChecker → PoseProgressChecker (`nav2_params_leo.yaml` only)

## Results vs. DWB baseline (run 4, same world, same explorer-adjacent fixes)

| Metric | DWB baseline | PR2 (RPP + shim) |
|---|---|---|
| Coverage @ 120 s | 76 m² | ~80 m² (43 m² @ 90 s vs 33 baseline) |
| Office interior mapped | 113 m² in ~16 min | ~97 m² interior + out-of-bounds areas in ~10 min of effective exploration |
| "Failed to make progress" | 13+ | 11 total — but ~9 of them while grinding at a single map pathology (see below); 0 in normal driving |
| Frontier goals sent | n/a (explore_lite) | 25 |
| Driving quality | oscillation-prone near obstacles | full-speed straight segments, clean rotate-then-drive behavior |

**Controller verdict: keep RPP + rotation shim.** In normal terrain it never
false-stalled and traversed the office noticeably faster.

## Two robustness findings (the real value of this run)

1. **Blacklist strike-memory bug (FIXED during the run).** Original TTL
   blacklist dropped expired entries entirely, so a goal failing every ~TTL
   never accumulated strikes and was retried forever — the rover ground at the
   office's fake SE wall opening indefinitely. Fix: expired entries stop
   blocking but are kept as strike memory (3×TTL), so the third failure
   triggers the 15-minute long ban. (`frontier_explorer.py::_prune_lists`)

2. **Planner-fail livelock (KNOWN ISSUE → fixed in PR3 code).**
   `husarion_office` has non-physical wall gaps (the lidar sees through them,
   the planner happily paths through). The rover drove out of the office
   interior through one, got physically stuck against out-of-bounds mesh
   geometry in lethal cost, and from there `ComputePathToPose` rejected every
   frontier — the explorer cycled validation-skips and the watchdog cleared
   costmaps in vain for the rest of the run (clearing a costmap cannot help a
   physically wedged robot). Escape requires *motion*, not replanning. The
   PR3 explorer adds a last-resort blind-reverse recovery when all frontiers
   are planner-rejected and the robot is stationary.

   The unreachable frontier cluster around x≈0–4, y≈−4..−9 in the log spam is
   the same pathology from the other side: visible-through-gap free space the
   planner could never reach.

## Artifacts

- `exploration.mp4` — full time-lapse (including the out-of-bounds excursion)
- `office_map.{pgm,yaml,png}` — interior fully mapped; smeared regions
  top-right are scan leakage through the non-physical gaps
- `coverage_curve.png`, `coverage.log`, `explorer.log`

## Verdict

Controller upgrade: **PASS** (adopted). Run also surfaced and fixed one
explorer bug and identified a second (fix included in PR3). Note for the
soak-test milestone: `husarion_office`'s fake openings make it a good
adversarial test world.
