# PR1 — Hardened frontier explorer: verification report

**Date:** 2026-06-11 · **World:** `leo_world` (20×20 m multi-room arena) ·
**Run:** headless sim, SLAM + Nav2 + `leo_rover_exploration` frontier_explorer

## What PR1 added

- Frontier goals snapped into known-free space (3×3 free block) and
  **pre-validated** with Nav2 `ComputePathToPose` before commitment
- Blacklist with **TTL (90 s) + strike escalation** instead of permanent bans
- **Goal hysteresis**: 10 s commit time + 25 % score margin before preempting
- **Stall ladder**: first stall clears Nav2 costmaps and retries; only a second
  stall blacklists
- **Watchdog**: recovers "frontiers exist but nothing happening" livelocks
- `respawn=True` on explorer/SLAM, `bond_respawn_max_duration` for Nav2
- Status as JSON on `~/status` (state, coverage, goal stats, distance)

## Results

| Metric | Value |
|---|---|
| Exploration time (sim) | ~10.5 min (630 s incl. return-to-start) |
| Final coverage | **390.8 m²** known of ≈396 m² reachable (≈99 %) |
| Frontier goals sent | 26 |
| Goals blacklisted | **0** (pre-PR1 run on same world: 4) |
| Pre-validation rejections ("no path") | 3 — unreachable frontiers skipped in ~1 s instead of a 60 s drive-and-fail cycle |
| Nav2 "Failed to make progress" | **0** |
| Watchdog recoveries | 1 (early SLAM-warmup livelock, auto-recovered) |
| Costmap-clear recoveries | 0 needed beyond watchdog |

## Artifacts

- `exploration.mp4` — time-lapse of the map being built, robot trail in red
- `exploration_final.png` — final frame: full coverage, trail through all rooms,
  rover returned to start (blue dot)
- `coverage_curve.png` — known/free/occupied area vs. sim time
- `coverage.log`, `explorer.log` — raw logs
- `leo_world_map.{pgm,yaml,posegraph,data}` — saved map + serialized pose-graph

## Verdict

PASS. All hardening features fired correctly in live conditions; the failure
modes observed before PR1 (goal aborts at inflation edges, permanent blacklist
loss, startup livelock) did not recur.
