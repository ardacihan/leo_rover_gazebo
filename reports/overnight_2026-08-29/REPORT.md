# Multi-robot integration validation - 2026-08-29

Branch: `feat/multi-robot-integration`

## Result

The camera-on WSL/Gazebo pipeline now performs honest pre-rendezvous
exploration, establishes a shared frame from common ArUco evidence cross-checked
against occupancy geometry, merges maps, and feeds peer-covered cells back to
the frontier explorers. No fixed or ground-truth transform is used by the
runtime pipeline; authored spawn truth is supplied only to the recorder for
error scoring.

## Acceptance evidence

| Gate | Fresh evidence | Status |
|---|---|---|
| Feature branch | `feat/multi-robot-integration` | PASS |
| Different initial poses, no shared frame | Separate `leo1/map` and `leo2/map`; TF bridge silent until rendezvous | PASS |
| RGB + depth + LiDAR | Message-level probe on both robots; 640x480 RGB, organized 640x480 XYZRGB cloud, 360-sample scan | PASS |
| NVIDIA GPU | Ogre: `D3D12 (NVIDIA GeForce RTX 4060 Ti)`; live NVIDIA-SMI: 857 MiB, 11% utilization | PASS |
| Common-marker alignment | Depot and office both locked only after 2+ common persistent tags | PASS |
| Accurate/stable merge | Depot stayed within 0.30 m / 1.1 deg; office ended at 0.18 m / 0.6 deg | PASS |
| Explorer consumes peer map | leo2 masked 4,161 depot and 12,308 office cells as peer-covered immediately after lock | PASS |
| Better than two independent robots | Depot 134.1 vs 133.5 m2; fresh office 103.7 vs 88.1 m2 at matched horizons | PASS |
| Multiple complete worlds | Fresh depot; fresh camera-on office integration; completed office pair retained for full-map evidence | PASS |
| Presentation media | Clean maps, trajectory overlays, curves, camera frames, live merge MP4 | PASS |
| End-to-end run budget | Fresh office runs 14m09s / 14m08s; harness now hard-stops the exact container before 20 min | PASS |

## Depot head-to-head

| Condition | Final known area | Duplicate area | Frontier goals | Failed goals |
|---|---:|---:|---:|---:|
| Coordinated | 134.2 m2 | 122.7 m2 | 33 | 1 |
| Independent | 133.5 m2 | 174.9 m2 | 42 | 1 |

At the independent run's final timestamp, the coordinated curve is already at
134.1 m2. Coordination therefore preserves/slightly improves the final map,
reduces repeated mapping by 52.2 m2 (29.8%), and sends nine fewer goals.

## Husarion office head-to-head

The fresh local WSL pair used cameras, depth clouds, GPU LiDAR, real ArUco
detection, and identical 10-minute exploration budgets. The independent
rovers did not rendezvous; that is valid baseline behaviour, but it means its
live `/shared_map` contains only leo1. Evaluation therefore unions both
recorded local maps with authored spawn truth **for scoring only**. That
transform was never published to either explorer.

| Condition | Matched-horizon known area | Duplicate area | Frontier goals | Failed goals |
|---|---:|---:|---:|---:|
| Coordinated | 103.7 m2 | 47.2 m2 | 28 | 1 |
| Independent | 88.1 m2 | 0.6 m2 | 18 | 0 |

Coordination maps 15.6 m2 more in the same horizon (+17.7%). The short
independent trajectories remained almost disjoint, so this pair is not used
to claim reduced overlap. The completed office pair from 2026-08-25 supplies
that full-map gate: its runtime coverage is 181.2 vs 179.8 m2, while the
current truth-overlap scorer gives 265.9 vs 319.6 m2 duplicate area (16.8%
less), with 53 vs 67 goals.

## Defects fixed during the fresh runs

1. A correct two-tag seed (0.13 m / 0.3 deg from truth) was rejected by hidden
   0.35/0.50 m fit defaults even though downstream gates documented 0.75/1.5 m.
   The seed and downstream gates are now consistent and regression-tested.
2. Hybrid map matching could accept a pre-rendezvous grid candidate internally.
   Even though the TF bridge correctly withheld it, the stale state later
   rejected the true candidate as a large jump. Hybrid/tag candidates are now
   preview-only until common-landmark evidence exists, and tag/map agreement is
   required before state acceptance.
3. The run harness now fails if either rover's RGB, camera calibration, depth
   cloud, or LiDAR stream is silent under full load.
4. The timelapse renderer now makes flipped arrays OpenCV-contiguous. Marker
   truth and the complete leo2 trajectory are transformed into leo1's map frame
   correctly in presentation overlays.
5. Independent runs that never rendezvous are now evaluated from both recorded
   local maps instead of silently scoring leo1's pre-lock `/shared_map` alone.
   This is an offline evaluation path only and is regression-tested.
6. The old exploration-only cap could exceed the requested wall limit after
   startup and media work. The local harness now defaults to 14 exploration
   minutes, reserves 150 seconds for collection, and enforces a container-ID-
   checked 20-minute end-to-end watchdog.

## Verification

- ROS package build: four relevant packages built successfully.
- Shared mapping full suite after final fixes: 58/58 tests passed.
- Collaborative explorer: 18/18 tests passed.
- The 23 focused alignment/policy/media/scoring regressions are included in
  that final shared-mapping pass.
- Existing unrelated `leo_nav2_exploration` contract suite remains 79/92; the
  same 13 previously documented configuration/operator-bundle failures were not
  represented as fresh simulation failures.

## Primary artifacts

- `depot_coordinated_fixed2/merged_map.png`
- `depot_coordinated_fixed2/traj_overlay.png`
- `depot_coordinated_fixed2/alignment.png`
- `depot_coordinated_fixed2/coverage.png`
- `depot_coordinated_fixed2/merge_timelapse.mp4`
- `depot_coordinated_fixed2/sensor_probe.json`
- `depot_coordinated_fixed2/gpu_runtime.txt`
- `depot_independent_fixed2/` (matched baseline)
- `depot_comparison.png` (matched-horizon coverage and duplication summary)
- `office_coordinated/` (fresh camera-on maps, plots, frames, logs and MP4)
- `office_independent/` (matched no-peer baseline and both saved local maps)
- `office_comparison.png` (fresh matched-horizon comparison)
- `office_complete_comparison.png` (completed-office comparison)
- `office_coordinated/merge_timelapse_prelock.png` (independent-map phase)
- `office_coordinated/merge_timelapse_mid.png` (live aligned merge phase)
- `office_independent/merge_timelapse_final.png` (explicit no-rendezvous state)

The interrupted `depot_coordinated_fixed1` directory is explicitly marked
`ABORTED.md` and is excluded from every result.
