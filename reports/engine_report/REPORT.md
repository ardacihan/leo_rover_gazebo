# Exploration engine verification — July 5, 2026

Full-stack headless runs (Gazebo Harmonic + slam_toolbox + Nav2 +
`leo_rover_exploration`), driven end-to-end by `scripts/auto_explore_run.sh`
(sim → SLAM/Nav2 → bootstrap jog → explorer → map save → teardown), all with
GPU rendering in WSL (RTF ≈ 1.2).

## Item-search runs (final algorithm: frontier → camera sweep → verify → return)

| World | Size | Items | Camera coverage | Sim time | Final map |
|---|---|---|---|---|---|
| `office_world` (new: corridor + 5 offices) | 24×16 m | **8/8** (≤2 cm err) | 87 % (exit by exhaustion) | ~115 min | clean |
| `leo_world` (benchmark arena) | 20×20 m | **6/6** | **90 % (target)** | ~70 min | clean |
| `depot_world` (new: small rooms) | 14×14 m | **4/4** | **90 % (target)** | ~55 min | clean, exactly 281×281 cells |

Every run terminated by itself, returned to its start pose, and serialized
map + pose graph. Figures: `final_runs_coverage.png`, `final_maps_board.png`.

## explore_lite vs. custom explorer (leo_world, identical stack + start jog)

- Raw mapping speed is a tie: both reach full coverage (~390 m²) in ~6 min
  (`explore_lite_vs_custom.png`).
- **Without** the 1 m start jog, explore_lite livelocks at startup forever:
  its first frontier goal spawns inside Nav2's goal tolerance, "succeeds"
  instantly without motion, and is re-sent in a tight loop
  (`comparison_run/explore_lite_livelock_no_bootstrap/`). The custom
  explorer is immune — goals that succeed without producing information are
  struck and replaced (PR4 machinery).
- explore_lite has no item search, no camera sweep, no planner-reject escape,
  and in an earlier run ground for 26+ min after full coverage without
  declaring completion. The custom explorer is the engine; explore_lite
  remains a speed baseline.

## SLAM robustness findings (the day's real lesson)

Three archived failure runs (`final_runs/depot_world_*_drift`,
`comparison_run/custom_drifted_slam`) plus the fix:

1. **Perceptual aliasing kills 2D scan matching.** The original depot layout
   (three identical parallel empty aisles) produced a wrong loop closure and
   a rotated duplicate map in every attempt, with light *and* dense scan
   matching. Room-and-doorway geometry of the same size is handled cleanly.
2. **Skid-steer in-place rotation is the odometry poison.** Yaw error
   accumulates fastest while spinning (wheel slip). Fixes that made all
   subsequent runs clean:
   `rotate_to_heading_angular_vel` 0.8→0.4, `max_angular_accel` 3.2→1.5
   (nav2), scan cadence 0.5 s→0.15 s, travel thresholds 0.2→0.1,
   `correlation_search_space_dimension` 0.5→0.8,
   `coarse_search_angle_offset` 0.349→0.523 (slam_toolbox).
3. Symptom signature for future debugging: known-area exceeding the world's
   reachable area and a growing map bounding box = drift; phantom
   "through-wall" frontiers are a *consequence*, and the explorer's
   long-ban/strike machinery contains them (it kept finding all items even
   on drifted maps).

## Real-robot readiness

The exploration layer is ready: goal validation, TTL blacklists with strike
escalation, stall ladder, bounded blind escapes, unproductive-goal striking,
sweep, verification, return-home — all exercised and effective across three
geometries with zero human intervention. The porting risks are *below* it:
real skid-steer odometry will be at least as bad as sim (keep rotation slow,
consider IMU fusion), corridor-heavy real environments need the strengthened
scan-matching settings, and the mock detector swaps for real ArUco detection
(interface already topic-based).
