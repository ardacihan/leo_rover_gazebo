# PR4 - Camera-coverage sweep + item search: verification report

**Date:** 2026-06-12  
**World:** `leo_world` (20 x 20 m multi-room arena)  
**Run:** headless sim, SLAM + Nav2 + `leo_rover_exploration` item search
(frontier explorer, camera sweep, and mock ArUco detector with 6 markers).

## What PR4 added

- **SWEEPING state:** after frontiers are exhausted, the explorer drives
  viewpoint goals until the RGBD camera (60 degree FOV, 3 m practical range)
  has observed at least 90% of wall cells. Lidar coverage alone does not find
  wall-mounted items.
- `CameraCoverageTracker`: frustum raycast onto the occupancy grid each 0.5 s,
  a world-quantized observed-cell set immune to SLAM re-anchoring,
  unobserved-wall clustering and viewpoint search, and a coverage grid on
  `~/camera_coverage`.
- **VERIFY state:** first detection of an item queues a standoff viewpoint to
  re-observe and confirm it; confirmed items are published on `~/found_items`
  as JSON and markers.
- `mock_aruco_detector`: simulates detections from ground-truth poses using
  FOV, range, and wall-normal checks until the real perception stack lands.

## Livelock found during the soak

The first sweep soak livelocked for about 3.5 h sim time: one goal at
`(-6.06, -6.10)` was sent 2,329 times while camera coverage stayed at 38.8%.
The rerun includes three fixes:

1. **Aim at a real cluster member.** The largest unobserved cluster was
   L-shaped, so its centroid sat on open floor and the wall remained outside
   camera range. Clusters now carry the member wall cell nearest the centroid
   as their target. Viewpoint search and goal yaw use that target, and
   viewpoint rings are capped at 0.8 times detection range.
2. **Check whether successful goals produced information.** If a completed
   sweep goal leaves its target unobserved, the target receives a strike in
   the TTL blacklist. The same rule handles frontier goals that succeed but
   leave their frontier in place, such as unknown slivers behind a wall.
3. **Do not resurrect sweep failures.** The watchdog blacklist reset now only
   runs in EXPLORING. It no longer revives unviewable sweep targets or blocks
   the intended sweep exit.

## Results

| Metric | Value |
|---|---|
| Frontier phase | ~20 min sim, 390.9 m2 lidar coverage, 30 goals |
| Sweep phase | ~53 min sim, camera coverage 38% to **90%** (3222/3570 wall cells) |
| Sweep goals | 22, **all distinct** (livelocked run: 2,329 sends of one goal) |
| Items found | **6 / 6 confirmed**, position error at most 0.02 m |
| Items needing the sweep | 3: ids 1, 2, 5; ids 3, 4, 6 were found during frontier exploration |
| Sweep-target strikes | 3 unproductive viewpoints pruned, no loops |
| Frontier-survived strikes | 2 through-wall slivers pruned |
| Stall ladder | 4 no-progress events, 2 escalated to blacklist |
| Watchdog costmap clears / blacklist resets | 5 / **0** |
| Escapes | 0, not needed in this world |
| End state | Returned to start; map and pose graph saved |

## Rebuild and rerun status

- The clean full PR4 rerun completed on June 12, 2026 and produced the metrics
  above. The source package is symlink-installed, so the run exercised the
  checked-out implementation.
- A subsequent full workspace rebuild completed all 13 packages.
- The later PR3 testability parameter keeps the same default Nav2 planner
  endpoint and does not change PR4 behavior.
- Repository-wide tests still include unrelated pre-existing failures noted
  in the PR3 report; the exploration package itself reports 0 test failures.

## Artifacts

- `exploration.mp4`, `exploration_final.png` - time-lapse and final frame
- `coverage_curve.png`, `coverage.log` - lidar coverage versus sim time
- `explorer.log` - full explorer log; search for `striking it` for the new
  self-pruning events
- `leo_world_map.{pgm,yaml,posegraph,data}` - saved map and pose graph
- `livelock_run/` - pre-fix evidence: the 2,329-send loop, 38.8% coverage
  flatline, final frame, and the frontier-sliver variant

## Verdict

**PASS.** The sweep terminates, item search completes at 6/6 with three items
found only through camera sweeping, and both successful-but-unproductive goal
classes now self-prune. Sweep viewpoints still use the common `_dispatch`
path, so `/exploration_claims` deduplication remains applicable.
