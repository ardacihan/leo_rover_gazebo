# PR3 - Last-resort escape recovery: verification report

**Date:** 2026-06-12  
**Worlds:** `husarion_office` and `leo_world`  
**Change under test:** blind reverse/rotate recovery after repeated planner
rejections while the rover is stationary.

## What PR3 added

- The watchdog tracks consecutive frontiers rejected by
  `ComputePathToPose`.
- At `escape_after_skips`, a stationary rover temporarily bypasses Nav2 and
  executes a bounded blind escape.
- Escapes alternate between reverse and rotate so repeated recoveries do not
  reproduce the same geometry.
- Normal frontier validation and Nav2 dispatch resume after the escape.
- `planner_action_name` makes the validation endpoint explicit and allowed a
  deterministic fault-injection test without replacing real navigation.

## Verification matrix

| Run | Trigger | Recovery | Evidence after recovery | Result |
|---|---|---|---|---|
| Natural `husarion_office` run | 7 consecutive real planner rejects while wedged near false wall openings | Blind reverse | New frontier `(-0.38, -4.77)` dispatched 6.5 s later; exploration continued | PASS |
| Deterministic `leo_world` run | Fault server returned empty paths; watchdog fired at 8 accumulated skips | Blind rotate | Validation was restored, then four real frontier goals were dispatched; known area reached 153.9 m2 | PASS |

The deterministic injector remained active past the trigger to prove that the
escape was bounded and did not disable planner validation. Once pass-through
was enabled, the explorer immediately returned to its normal dispatch path.

## Natural-run observations

- Known map area grew from 15.6 m2 at 30 s to 100.6 m2 at the 540 s snapshot.
- The exact recovery sequence is preserved in `office_explorer.log`:
  planner rejects, `blind reverse escape`, then two new frontier goals.
- This reproduces the PR2 failure mode using the real Nav2 planner and the
  adversarial non-physical gaps in `husarion_office`.

## Deterministic-run observations

- The trigger occurred at 8 accumulated skips and selected the rotate branch.
- After validation was restored, goals were dispatched at
  `(-1.51, 1.40)`, `(1.17, -1.81)`, `(-3.85, -4.36)`, and `(7.40, -2.03)`.
- Known area rose from 42.9 m2 before resumed navigation to 153.9 m2.
- The recorder captured 265 frames and produced a stable MP4/final PNG.

## Build and test status

- Full workspace rebuild: **PASS**, 13 packages completed.
- `leo_rover_exploration` package test invocation: 0 tests, 0 failures.
- The full repository test invocation reported 50 failures outside this
  package, primarily missing `map_merge` fixture maps and existing
  `leo_rover_control` flake8 findings. These are not PR3/PR4 regressions.
- ROS external shutdown is now handled as a normal exit in the explorer and
  verification helpers, preventing false traceback noise in future runs.

## Artifacts

- `office_escape.mp4`, `office_escape_final.png` - natural office recovery
  snapshot and time-lapse
- `office_explorer.log`, `office_coverage.log` - real planner evidence
- `fault_injection_final/exploration.mp4` and
  `fault_injection_final/exploration_final.png` - deterministic run
- `fault_injection_final/explorer.log`,
  `fault_injection_final/rejector.log`, `status.log`, `cmd_vel.csv`,
  `odom.csv`, and `coverage.log` - raw acceptance evidence

## Follow-up: ghost-grind livelock found in the office soak (2026-06-12 evening)

A long unattended office soak (`ghost_grind_run/`, plus the archived
`exploration_soak.mp4` / `exploration_soak_final.png`) exposed a second,
slower livelock that the original PR3 escape could not break:

- The office world's non-physical wall gaps let the lidar map "ghost"
  geometry behind the east wall (the black speckle/ray fans in the soak
  video). Those ghost frontiers are visible to the lidar but permanently
  unreachable for the planner.
- Skip-strike escalation put ghosts into the blacklist at **1 strike /
  90 s TTL**. Reaching the reset-proof long ban required 3 separate
  escalations ~90 s apart, but in the stationary endgame the watchdog
  blacklist reset fires every ~2 minutes and wiped every 1-2-strike entry
  first. Ghost entries could never mature; skip strikes climbed past 17
  while coverage stayed flat at 142.6 m2 for 90+ minutes.
- The long-horizon wedge escape also fired on the *idle* endgame robot
  (stationary because everything was banned), and every escape re-expired
  the skip list, restarting the churn.

### Fix

1. `_add_skip` escalation now enters the blacklist at
   `blacklist_max_strikes` directly: three planner rejections are already
   the proof of unreachability, so the entry gets the 900 s long ban
   immediately and survives watchdog resets (log message changed to
   `long-banning`).
2. The long-horizon wedge escape only fires while work is active
   (navigating or validation in flight); a stationary rover with nothing
   eligible is finishing, not wedged.

### Verification (`ghost_fix_run/`)

Fresh full office run with the fixed code, same headless stack:

- 98 min sim total: frontier exploration, camera sweep, return to start,
  `Exploration finished.`, and a saved map bundle - the soak never got past
  flat coverage in 100+ min.
- 109 ghost goals long-banned, 23 bounded escapes, 8 blacklist resets -
  and the resets no longer revived confirmed ghosts.
- Camera sweep terminated at 25% wall coverage by design: the denominator
  counts ghost wall cells behind the real walls, which no viewpoint can
  observe; each was struck once and the sweep exited instead of looping.
- Final map (`exploration_final.png`, `office_map.pgm`): full office
  interior with crisp walls; ghost bleed is limited to one small ray fan
  at the breach corner instead of the soak's large speckle field. The
  speckle itself is world geometry (lidar through non-physical gaps), not
  an algorithm or SLAM defect - `leo_world` runs stay clean.

## Verdict

**PASS.** PR3 breaks the planner-rejection livelock with physical motion,
returns control to normal validated navigation, and works through both
alternating escape branches across two independent runs. The follow-up
ghost-grind livelock is fixed and verified end-to-end: the office run now
terminates, returns home, and saves its map.
