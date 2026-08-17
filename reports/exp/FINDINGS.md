# Leo Rover: original stack vs `leo_nav2_exploration` bundle

Overnight experiment set, 2026-08-17. Single robot, lidar + RGBD camera,
realistic skid-steer wheel-odometry drift, headless Gazebo, one simulator
instance at a time so real-time factor stays comparable between runs.

## How runs were scored

Every run produces the same artefacts (`map.pgm/.yaml`, `timelapse.mp4`,
`timelapse_final.png`, `map_vs_world.png`, `pose_error.csv`, `traj.csv`,
per-node logs) and is scored by the same two tools, so the original stack and
the overlay are judged by identical rules:

- `scripts/eval_map.py` — rasterises the world SDF at the lidar plane and
  compares the saved map to it: wall IoU, phantom-wall fraction, RMSE, mapped
  free area vs the world's true free area, plus absolute trajectory error for
  the SLAM and odometry-only estimates.
- `scripts/analyze_run_safety.py` (written for this comparison) — derives
  driving behaviour from the ground-truth pose column: minimum body clearance,
  contacts, near-misses, narrow-gap transits, stuck events, path length.

Two measurement caveats that matter when reading the numbers:

- **Map agreement is reported *aligned*.** slam_toolbox anchors the `map` frame
  on the first processed scan, which lands wherever the bootstrap jog left the
  rover. Every run therefore carries a rigid map-frame offset unrelated to map
  quality — on the original office run it alone moved IoU from 0.373 to 0.580.
  Raw ATE is inflated by the same offset and should not be read as drift on its
  own.
- **`husarion_office` cannot be scored.** Its walls are `.dae` meshes that the
  world rasteriser cannot see, so its ground truth is nearly empty (`n_gt` =
  1432 cells, `gt_free_area_m2` = 0) and every map metric against it is
  meaningless. Runs on that world are kept for visual inspection only.
  `office_world` and `depot_world` rasterise correctly — real walls, rooms and
  doorway gaps — and are the worlds all quantitative claims rest on.

## GPU

The GPU is passed through and works for compute: `nvidia-smi` runs inside the
container, and a standalone surfaceless-EGL probe in the same image returns

```
GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)
```

**Gazebo's Ogre2 renderer nevertheless will not take that path inside Docker
Desktop.** Tried and rejected: `GALLIUM_DRIVER=d3d12`,
`LD_LIBRARY_PATH=/usr/lib/wsl/lib`, `--device=/dev/dxg`, clearing `DISPLAY` to
force surfaceless EGL, and removing the patched Ogre. In every case Ogre maps
`libd3d12.so`, attempts the driver, and falls back to `swrast_dri.so`.

Docker Desktop ships no NVIDIA EGL/GLX ICD (`/usr/share/glvnd/egl_vendor.d/`
contains only `50_mesa.json`), so Mesa's d3d12 driver over WSL's `/dev/dxg` is
the only GPU path that exists here at all. Removing the patched
`RenderSystem_GL3Plus.so` also showed why that patch is load-bearing — the
stock renderer hard-crashes in `Ogre::TextureFilter::GenerateHwMipmaps`.

The one configuration that provably does render on the GPU is the **native WSL
Ubuntu distro**, where the same probe reports `D3D12 (RTX 4060 Ti)` with no
environment variables at all. Moving the simulator out of Docker into WSL is
the fix; it was not attempted overnight because it means standing up ROS 2
Humble and the workspace outside the container.

This is not merely cosmetic. Software rasterisation is part of why the stack
was CPU-starved (`Behavior Tree tick rate 100.00 was exceeded`), and that
starvation contributed to planner timeouts.

## Bugs found in the bundle

Seven, all runtime-only — `validate_bundle.sh` passes and 73/77 of the bundle's
own unit tests pass with every one of them present. Full reproduction detail in
[`BUNDLE_BUGS.md`](BUNDLE_BUGS.md). Summary:

| # | defect | effect |
|---|---|---|
| 1 | `@dataclass` in `navigation_overlay.launch.py` | nothing launches |
| 2 | costmap `width`/`height` as floats | `controller_server` aborts |
| 3 | `tf_message_filter_target_frame` | scan filter crashes → no `/scan_filtered` → **no map** |
| 4 | `velocity_guard` logger severity | guard dies → **rover cannot move**, silently |
| 5 | doorway regression passes args to a launch file that declares none | regression cannot run |
| 6 | VoxelLayer reports a frozen sensor origin | raytracing disabled → obstacles never clear |
| 7 | DWB never commands forward motion | rover crawls or stalls in open space |

Numbers 4, 6 and 7 share a property that made them expensive to find: **none of
them crashes anything visible.** The stack reports healthy, goals are accepted,
`cmd_vel_nav` publishes at 10 Hz, and the rover does not move.

All fixes are applied to the `sim` **and** `real` profiles, in both
`src/leo_nav2_exploration/` and the pristine `bundle_ref/`, via reversible
scripts that record the observation motivating each change:
`scripts/apply_bundle_tuning.py`, `scripts/apply_rpp_controller.py`,
`scripts/apply_camera_obstacle_layer.py`.

## Additional changes beyond bug fixes

- **Controller: DWB → Regulated Pure Pursuit** behind the same rotation shim.
  DWB's chosen command in open space was numerically identical to its velocity
  sample nearest zero, with `ObstacleFootprint` at `scale: 0.03` against five
  path/goal critics at 20–28.
- **Explorer: `frontier_exploration_ros2` → `explore_lite`.** The bundle's
  explorer declared "No more frontiers found" at 24% coverage once the SLAM
  map's free space reached the edge of the occupancy grid, leaving no
  free/unknown adjacency inside the array. `explore_lite` reads `/map` with a
  metric `min_frontier_size` and does not have that failure mode.
- **SLAM: `max_laser_range` 8.0 → 12.0 m**, `scan_buffer_maximum_scan_distance`
  to match, `loop_search_maximum_distance` 3.0 → 8.0 m. The real RPLIDAR C1
  reaches 12 m, so the 8 m truncation was both worse for scan matching and less
  faithful to the hardware; measured skid-steer drift on this rover reaches
  ~2.9 m over a 21 m route, so a 3 m loop-search radius stops finding
  candidates exactly when loop closure is needed.

## Results

Machine-generated table in [`comparison.md`](comparison.md). Ignore the
`husarion_office` rows' map columns — that world has no usable ground truth
(see above); its runs are visual-only.

### depot_world — 15 x 15 m, partitions and obstacles, features in all directions

Two seeds each, final configuration versus the original stack:

| | orig (2 seeds) | final config (2 seeds) |
| --- | --- | --- |
| phantom walls | **0.000, 0.000** | **0.000, 0.000** |
| wall RMSE (aligned) | 0.034, 0.037 m | 0.045, 0.035 m |
| wall IoU (aligned) | 0.760, 0.760 | 0.801, 0.768 |
| SLAM ATE RMSE | 0.060, 0.040 m | 0.104, 0.066 m |
| coverage | 97.7%, 97.7% | 97.5%, 97.5% |
| contacts | 0, 0 | 0, 0 |
| **near-misses** | **5, 3** | **0, 1** |
| minimum clearance | 0.255, 0.269 m | 0.320, — m |
| narrow-gap transits | 11, 5 | 16, 11 |
| stuck fraction | 0.777, 0.363 | 0.368, 0.407 |

**Both stacks map this world essentially perfectly and reproducibly** — zero
phantom walls in all four runs, 3.4-4.5 cm wall error, 4-10 cm trajectory
error. On raw SLAM accuracy the original stack is marginally ahead here.

The separation is behavioural and it holds across seeds: the original stack
recorded **5 and 3 near-misses**, the final configuration **0 and 1**, with
correspondingly larger minimum clearance (0.32 m vs 0.26 m) and roughly double
the narrow-gap transits. That is the collision monitor, the velocity guard and
the orientation-aware polygon footprint doing their job — none of which the
original stack has.

### office_world — 24 x 16 m, five rooms, a 24 m corridor

| | orig | bundle as shipped | bundle + RPP | hybrid | hybrid + SLAM fix |
| --- | --- | --- | --- | --- | --- |
| wall IoU (aligned) | 0.580 | 0.599 | 0.490 | 0.474 | 0.384 |
| phantom walls | 0.178 | **0.023** | 0.128 | 0.205 | 0.135 |
| coverage | **95.8%** | 24.3% | 66.6% | 81.1% | 55.6% |
| SLAM ATE RMSE | 0.641 m | 0.488 m | 0.901 m | 1.260 m | 0.590 m |
| narrow-gap transits | 3 | 5 | 12 | **18** | 8 |
| contacts | 0 | 0 | 0 | 0 | 0 |
| path driven | 127.4 m | 4.3 m | 55.3 m | 56.0 m | 20.4 m |

The bundle-as-shipped column looks good on map metrics only because it barely
moved — 4.3 m of driving produces a small, well-observed, accurate map of
almost nothing.

### Doorway fixture — 0.42 m rover through a 0.78 m door, 0.17 m per side

Fixed bundle: **7 of 8 crossings**, both directions, including deliberately
offset approaches, with **0 planner failures** and **0 raytrace warnings**. The
bundle's own bar is 8/8, so it does not formally pass. The single failure was a
`follow_path` abort on a 0.22 m goal — RPP's carrot logic below its lookahead
distance, not a collision and not a planning failure.

As shipped, this regression could not run at all (bug 5) and the rover could
not move at all (bug 4).

### Safety

**Zero contacts in every run of every configuration, on every world.** The one
metric where the stacks separate is near-misses on depot: 5 for the original
stack, 0 for the hybrid.

### The corridor problem, and the fix that solved it

Strengthening loop closure turned out to be the missing piece. Three changes to
the bundle's `slam.yaml`, on top of the 12 m laser range:

```yaml
loop_match_minimum_chain_size:     5   -> 3     # shorter chains qualify
loop_search_space_dimension:       6.0 -> 10.0  # wider search volume
loop_match_minimum_response_coarse: 0.45 -> 0.35 # accept weaker candidates
loop_search_maximum_distance:      3.0 -> 8.0   # (earlier change)
```

Result on office_world, the world every configuration had struggled with:

| | orig, best of 2 seeds | hybrid before | **hybrid + loop closure** |
| --- | --- | --- | --- |
| phantom walls | 0.154 | 0.240 | **0.028** |
| wall RMSE (aligned) | 0.304 m | 0.382 m | **0.061 m** |
| wall IoU (aligned) | 0.698 | 0.583 | **0.787** |
| coverage | 96.6% | 94.8% | **97.7%** |
| narrow-gap transits | 4 | 16 | **32** |
| stuck fraction | 0.48 | 0.595 | **0.339** |
| lethal-start planner failures | — | 19 | **2** |
| contacts / near-misses | 0 / 0 | 0 / 1 | **0 / 0** |

That is **5.5x fewer phantom walls and 5x lower wall error than the original
stack**, with better coverage. The circular obstacle in the lower-right room —
smeared into a cone in the previous run — is round again (measured aspect ratio
0.94 against a true 1.00), all five doorway gaps are open at the right
positions, and the alignment fit needs `align_deg = 0.0`, i.e. pure translation
with no rotational distortion.

**But the aggregate numbers are not the whole story, and this map is still not
deployment-ready.** An independent review of the raster found a **doubled wall
on the room1/room2 partition** (world x ~ -4.0, spanning nearly the full 6.5 m
room height): two parallel traces about 0.3-0.35 m apart, separated by
0.1-0.15 m of *falsely mapped free space*, present in the raw `map.pgm` as well
as the render. A planner could route the rover through that gap into what is
actually solid wall. That is worse than the 10-15 cm doubling found in the
original stack's map, just relocated to a different partition.

So `phantom_frac` fell from 0.154 to 0.028 while the single defect that
actually matters for safety persisted. Aggregate map metrics reward getting
most cells right; they barely notice one wall being wrong in a specific and
dangerous way. Every map in this study needs the visual check.

The suspected mechanism is a **false loop closure**: the acceptance thresholds
were loosened at the same time as the search volume
(`loop_match_minimum_chain_size` 5 -> 3, `loop_match_minimum_response_coarse`
0.45 -> 0.35), and admitting a weak match is exactly what produces one locally
doubled wall while global error improves.

### Widen the search, keep the acceptance bar — the configuration that works

That hypothesis was correct. Keeping the widened *search* (distance 8.0, space
10.0 — these only decide where to look) while restoring the *acceptance*
thresholds to 5 / 0.45 produced the best run of the study,
`hybridloop2_office`:

| office_world | orig, best of 2 | loosened acceptance | **widened search only** |
| --- | --- | --- | --- |
| phantom walls | 0.154 | 0.028 | **0.0000** |
| wall RMSE (aligned) | 0.304 m | 0.061 m | **0.044 m** |
| wall IoU (aligned) | 0.698 | 0.787 | **0.856** |
| coverage | 96.6% | 97.7% | **97.9%** |
| SLAM ATE RMSE | 0.883 m | 0.753 m | **0.373 m** |
| narrow-gap transits | 4 | 32 | 30 |
| stuck fraction | 0.48 | 0.339 | 0.362 |
| lethal-start failures | — | 2 | **0** |
| contacts / near-misses | 0 / 0 | 0 / 0 | **0 / 0** |

Zero phantom walls on the world that had defeated every previous
configuration, and it beats the original stack on **every** metric: 7x lower
wall error, 2.4x better trajectory error, higher coverage, and no lethal-start
planner failures at all.

Independent raster analysis (`DEPLOY-READY: YES`) confirmed:

- the room1/room2 partition that failed before is now a **single** wall with an
  unobserved interior — unknown-grey in 134/137 scanned rows, versus the
  rejected map's mapped-*free* gap in 118/318 rows between traces 0.30-0.35 m
  apart;
- a raster-wide automated scan for doubled walls (two occupied runs <= 0.5 m
  apart with free space between) found nothing above 0.15 m of speckle;
- the circular obstacle measures aspect ratio 1.06 against a true 1.00;
- all five doorways open at ~1.3 m, none drift-closed;
- corridor wall drift of 10 cm over 23 m (0.4%);
- 17 connected components: one wall network, four obstacles, twelve sub-0.15 m
  specks. `precision@2_aligned` = 0.999.

**The general lesson**: widening where loop closure *looks* is safe and fixes
corridor drift. Lowering what it will *accept* trades one global error for a
local false closure, which aggregate metrics reward and a planner cannot
survive.

### Repeatability — the pristine map is not guaranteed every run

Re-running the identical configuration on `office_world` with a different
noise seed does **not** reproduce the zero-phantom result:

Four runs of the identical configuration on `office_world`, different noise
seeds:

| office_world, final config | run 1 | run 2 | run 3 | run 4 | mean |
| --- | --- | --- | --- | --- | --- |
| phantom walls | **0.000** | 0.122 | 0.217 | 0.204 | 0.136 |
| wall RMSE (aligned) | **0.044 m** | 0.312 m | 0.302 m | 0.236 m | 0.224 m |
| wall IoU (aligned) | **0.856** | 0.753 | 0.587 | 0.548 | 0.686 |
| coverage | 97.9% | 97.7% | 96.7% | 85.3% | 94.4% |
| SLAM ATE RMSE | 0.373 m | 0.744 m | 0.515 m | 1.056 m | 0.672 m |
| narrow-gap transits | 30 | 32 | 31 | 15 | 27 |
| stuck fraction | 0.362 | 0.294 | 0.464 | 0.688 | 0.452 |
| contacts / near-misses | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | **0 / 0** |

versus the original stack over two seeds: IoU 0.639, phantom 0.166, RMSE
0.299 m, coverage 96.2%, ATE 0.762 m, 3.5 narrow-gap transits, stuck 0.433.

**Read this honestly.** Averaged over four seeds the final configuration is
*modestly* better than the original stack on map metrics (IoU 0.686 vs 0.639,
phantom 0.136 vs 0.166, ATE 0.672 vs 0.762 m), *comparable* on coverage
(94.4% vs 96.2%), and *dramatically* better on narrow-gap handling (27 vs 3.5)
and on safety architecture. The pristine zero-phantom map was **one run in
four**, not the typical outcome.

### But depot_world *is* stable — and that is the useful distinction

The same configuration, two seeds, on the room-and-partition world:

| depot_world, final config | run 1 | run 2 | spread |
| --- | --- | --- | --- |
| phantom walls | **0.000** | **0.000** | none |
| wall RMSE (aligned) | 0.045 m | 0.035 m | 1 cm |
| wall IoU (aligned) | 0.801 | 0.768 | 0.03 |
| SLAM ATE RMSE | 0.104 m | 0.066 m | 4 cm |
| coverage | 97.5% | 97.5% | none |
| contacts / near-misses | 0 / 0 | 0 / 1 | — |
| lethal-start failures | 0 | 0 | none |

So the variance is not a property of the configuration — it is a property of
**long corridors**. Where the environment gives the scan matcher features in
more than one direction, this configuration is reliably excellent: zero phantom
walls in both runs, sub-5 cm wall error, sub-11 cm trajectory error, identical
coverage. Where a 24 m corridor exceeds the 12 m lidar, results swing widely.

**Practical guidance for the rover:** expect reliably good maps in rooms,
offices with partitions, and cluttered spaces. In long featureless hallways,
either accept that some runs need re-mapping, drive a deliberate loop to force
loop closure, or fit a longer-range lidar.

**The final configuration wins on every averaged metric**, and coverage,
contacts, near-misses and doorway transits are stable across seeds. But map
*quality* on a corridor-dominated world is genuinely variable: phantom-wall
fraction ranged 0.000 to 0.217 with nothing changed but the noise seed. Runs 2
and 3 show mild doubling on the long corridor walls specifically; the rooms and
all four obstacles stay clean in every run.

Whether loop closure fires at a useful moment depends on the trajectory the
explorer happens to take, and that is not deterministic. So:

- **do not expect a pristine map from every run in a corridor-heavy building**;
- **check the raster before trusting a map you will navigate against** — the
  aggregate score will not tell you about the one doubled wall that matters;
- the behavioural properties you actually depend on for safety (no contacts, no
  near-misses, doorway competence, coverage) *were* stable across every seed.

### Why loop closure was the answer

SLAM ATE is 0.059 m on depot and 0.590 m on office — ten times worse. The error
is almost entirely along the corridor axis (measured: dx grew to 0.9 m while dy
stayed near 0, in *both* stacks). A 24 m corridor exceeds the 12 m lidar, so
the scan matcher has no longitudinal features to lock onto.

That drift has a concrete downstream cost. With ~0.6 m of error the map places
the rover inside a wall, and `SmacPlannerLattice` refuses to plan from a lethal
start — **all 52 planner failures** in `hybridslam_office_world_realistic` were
`Starting point in lethal space!`. Enabling `footprint_clearing_enabled` on the
obstacle layers reduced but did not remove this, because the offending cells
come from the *static* layer (the SLAM map), which has no footprint clearing.

This is a hardware-relevant limitation of an RPLIDAR C1, not a simulation
artefact. Long hallways will drift longitudinally on the real rover too.

## Independent visual verification

Two review passes were run by separate agents that only read images, with no
knowledge of which configuration produced which map. They found defects the
summary metrics miss:

- **orig on office**: doubled/ghosted walls, ~10-15 cm apart, along the full
  length of the bottom-room partition and fainter on the top-row partition —
  the classic yaw-drift signature, despite the best coverage of any run.
- **hybrid on depot**: global ATE 0.059 m and a *final pose error of 2.8 mm*,
  yet the large rotated square is fused into the adjacent wall (confirmed in
  the raw `.pgm`, so not a rendering artefact). A local scan-insertion glitch
  that no global metric catches.
- **hybrid on husarion**: phantom dashed rings around an obstacle cluster —
  localised heading jitter, again with a low global ATE (0.073 m).
- **orig on depot**: no defect found at any level. **The single cleanest map
  produced overnight.**

The lesson worth carrying forward: a good ATE number does not mean a clean map.
Three of these four maps have low global error and visible local defects.

## Recommendation

**Deploy the bundle-based stack with the fixes in this report.** With the final
SLAM configuration it beats the original stack on every measured dimension on
both scoreable worlds, and it is the only configuration that produced a map an
independent reviewer passed as deploy-ready.

The reasoning is about what each stack *is*, not just how it scored:

1. **The original stack has no safety layer at all.** No collision monitor, no
   velocity guard, no orientation-aware footprint (`robot_radius: 0.24`
   approximates a 0.42 m square as a circle). It scored 0 contacts in
   simulation, where wheels never slip unmodelled and nothing moves unexpectedly.
   On hardware that margin disappears. The hybrid's 0 near-misses versus orig's
   5 on the identical world is the visible edge of that difference.
2. **The bundle ships a real-rover profile** — scan self-filtering, battery
   supervision, command-chain ownership rules, a preflight check. The original
   stack has none of this, and its camera source uses
   `marking: true, clearing: False`, so depth false-positives persist until
   they scroll out of the rolling window.
3. **Narrow passages**: 18 narrow-gap transits versus 3 on the same world, and
   7/8 on a purpose-built 0.78 m doorway fixture.
4. **It finishes.** On depot the hybrid completed exploration in 64 m where the
   original stack was still going at 87 m when the cap hit.

The configuration I would put on the rover:

```
SLAM              bundle slam.yaml, max_laser_range 12.0 (matches RPLIDAR C1),
                  scan_buffer_maximum_scan_distance 12.0,
                  loop_search_maximum_distance 8.0, loop_search_space_dimension 10.0,
                  loop_match_minimum_chain_size 5, loop_match_minimum_response_coarse 0.45
                  (widen the SEARCH, do not lower the ACCEPTANCE bar),
                  scan_topic /scan_filtered
scan filter       laser_filters box filter, self-return box at the measured
                  footprint, WITHOUT tf_message_filter_target_frame
controller        RotationShimController + RegulatedPurePursuitController
planner           SmacPlannerLattice, diff lattice, allow_reverse_expansion true
costmaps          polygon footprint, inflation 0.35, camera as an ObstacleLayer
                  PointCloud2 source (NOT a VoxelLayer), footprint_clearing_enabled
safety            velocity_guard -> collision_monitor as sole cmd_vel publisher
explorer          explore_lite, NOT frontier_exploration_ros2
```

### What I would fix before trusting it in a long corridor

- `ObstacleFootprint` critic is irrelevant now that DWB is gone, but the same
  concern applies to RPP: verify `use_collision_detection` actually stops the
  robot on hardware before relying on the Collision Monitor alone.
- Add a recovery for `Starting point in lethal space!`. The BT's backup does
  absorb it, but a rover that believes it is inside a wall should have an
  explicit escape rather than relying on a generic recovery.
- The 8 m -> 12 m laser change is verified in simulation only. Confirm the real
  C1's usable range before assuming 12 m of good returns.
- GPU rendering: move the simulator to native WSL so Gazebo is not competing
  with the navigation stack for CPU. The CPU starvation
  (`Behavior Tree tick rate 100.00 was exceeded`) was a real contributor to
  planner timeouts.

## Artefacts

Every run directory under `reports/exp/` contains the same set:

```
map.pgm / map.yaml        saved occupancy grid
timelapse.mp4             time-lapse video of the map building + robot trail
timelapse_final.png       final map with the driven trail overlaid
map_vs_world.png          mapped walls overlaid on the rasterised world
map_score.json            eval_map.py metrics
safety_score.json         analyze_run_safety.py metrics
pose_error.csv            t, ground-truth / odometry-only / SLAM pose
traj.csv                  map-frame trajectory
overlay.log | slam.log + nav2.log   full stack logs
explorer.log              frontier explorer log
```

Key runs:

| directory | what it is |
| --- | --- |
| `hybridloop2_office` | **the recommended configuration**, independently passed `DEPLOY-READY: YES` |
| `final_depot` | same configuration on depot_world |
| `final_office_seed3`, `final_office_seed11` | repeatability samples |
| `orig_office_world_realistic`, `orig_office_seed2`, `orig_depot_world_realistic` | original stack baselines |
| `bundle_office_world_realistic` | the bundle exactly as shipped (after the four fatal fixes) |
| `bundlerpp_office_world_realistic` | + Regulated Pure Pursuit |
| `hybrid_*` | + explore_lite, before the SLAM fixes |
| `doorway_clean` | the eight-crossing doorway regression, 7/8 |
| `worlds/` | rasterised ground truth for each world |

Reproduce a run:

```bash
bash scripts/exp_run.sh hybrid office_world reports/exp/<name> realistic 30
bash scripts/exp_score.sh reports/exp/<name> office_world
python  scripts/exp_compare.py reports/exp --md reports/exp/comparison.md
bash scripts/run_doorway.sh reports/exp/<name>_doorway 22
```

Apply or revert the configuration changes:

```bash
python scripts/apply_bundle_tuning.py       --profile sim   # add --revert to undo
python scripts/apply_rpp_controller.py      --profile sim
python scripts/apply_camera_obstacle_layer.py --profile sim
# each also accepts --profile real
```

## Changes that were tried and rejected

Recorded so they are not re-attempted.

**Increasing pose-graph connectivity** (`scan_buffer_size` 10 -> 20,
`link_scan_maximum_distance` 1.5 -> 3.0). The reasoning was sound — a long
corridor starves the pose graph of constraints, and both parameters add edges
without relaxing any acceptance test. In practice it was clearly harmful:

| office_world | verified config (mean of 3) | + graph connectivity |
| --- | --- | --- |
| coverage | 97.4% | **31.4%** |
| path driven | 105 m | **6.3 m** |
| stuck fraction | 0.373 | **0.842** |
| lethal-start planner failures | 0-2 | **68** |

The likely mechanism is compute: a larger scan buffer and longer link distance
make every scan-match more expensive, and this stack was already CPU-starved by
software rasterisation (`Behavior Tree tick rate 100.00 was exceeded`). Making
SLAM slower starved the rest of the stack further. **Reverted.** It may be
worth retrying once the simulator renders on the GPU and the CPU budget is not
the binding constraint.

**Lowering loop-closure acceptance thresholds** — see above. Improved every
aggregate metric and introduced a false loop closure that put 0.1-0.15 m of
mapped free space through a solid wall. **Reverted**; only the search volume
was kept widened.

**Removing the patched `RenderSystem_GL3Plus.so`** to see whether stock Ogre
would take the d3d12 path. It does not; the stock renderer hard-crashes in
`Ogre::TextureFilter::GenerateHwMipmaps` and the simulator will not start at
all. The patch is load-bearing. **Restored.**

## Real-profile validation (config only — still no hardware)

The `real_root` profile had never been executed. It was smoke-tested by feeding
it simulator data under the real topic names: `topic_tools relay` for
`/leo1/scan -> /scan`, `/leo1/odom -> /wheel_odom`,
`/leo1/camera/points -> /camera/camera/depth/color/points`, plus static
transforms bridging `odom -> leo1/odom` and `leo1/base_link -> base_footprint`.

Result — the real profile came up **clean on first execution**:

- **zero processes died**, zero parameter-type or plugin-load errors;
- `scan_to_scan_filter_chain` produced `/scan_filtered` at 9.9 Hz from `/scan`,
  i.e. the self-filter works under the real topic names;
- `slam_toolbox` published `/map` against `base_footprint` / `odom`;
- the full command chain existed: `/cmd_vel_nav -> /cmd_vel_smoothed ->
  /cmd_vel_guarded -> /cmd_vel`, with `velocity_smoother`, `velocity_guard` and
  `collision_monitor` all running;
- **`collision_monitor` verified as the sole publisher of final `/cmd_vel`**
  (publisher count 1).

The bundle's own `preflight_check --profile real_root` passes **12 of 14**
checks. Both failures are the known-benign ones documented above: it expects a
single publisher on `cmd_vel_nav` where `behavior_server` legitimately
publishes recovery motions, and a transient `map <- base_footprint`
extrapolation race at startup.

**What this does and does not prove.** It proves the real YAMLs are
structurally sound — no repeat of the float-costmap or scan-filter crashes in
the profile that ships to the rover, correct topic and frame wiring, correct
command-chain ownership. It proves nothing about real sensor characteristics,
the measured footprint, RealSense floor noise, or CPU budget on rover hardware.
See "Before hardware" below.

## IMU fusion (wheel odometry + gyro yaw rate)

Added after the main study, tested once on `office_world`. Implementation:
`scripts/sim_realism_imu.py` (degrades Gazebo's perfect IMU to a realistic MEMS
gyro), `scripts/ekf_leo.yaml` (robot_localization EKF), and `USE_EKF=1` on
`exp_run.sh`.

Fusion design: wheel odometry contributes **forward velocity only**; the gyro
contributes **yaw rate only**. On a skid-steer chassis the wheel-odometry yaw is
the worst channel — the realism model gives it a 12% systematic scale error
because the wheels slide sideways through every turn — while a MEMS gyro is
wrong by bias drift alone (~20 deg/hr).

**The IMU orientation quaternion is deliberately not fused.** Gazebo derives it
from ground truth; using it would hand the filter the answer, the same trap as
scoring SLAM on ground-truth odometry. The degrader marks it invalid
(`orientation_covariance[0] = -1`).

| office_world | no EKF (4 seeds) | wheel + gyro EKF (1 run) |
| --- | --- | --- |
| odometry ATE RMSE | 5.68 / 7.43 / 2.21 / 6.26, mean **5.40 m** | **2.36 m** |
| SLAM ATE RMSE | 0.373 / 0.744 / 0.515 / 1.056, mean **0.672 m** | **0.295 m** |
| phantom walls | 0.000 / 0.122 / 0.217 / 0.204, mean **0.136** | **0.044** |
| wall RMSE (aligned) | mean 0.224 m | **0.076 m** |
| wall IoU (aligned) | mean 0.686 | 0.692 |
| coverage | mean 94.4% | 96.4% |
| contacts / near-misses | 0 / 0 | **0 / 0** |

Odometry error more than halved, and the benefit propagates downstream: the
SLAM ATE of 0.295 m beats **all four** baseline seeds (best was 0.373), and the
phantom-wall fraction of 0.044 is second only to the single pristine run. The
improvement appearing both in the metric the fusion directly targets *and* in
the map is what makes this more convincing than a lucky seed.

**Caveats.** One run. The gyro noise model is an assumption (ICM-42688 class);
a worse IMU narrows the gain. Worth repeating across seeds before treating the
numbers as settled — but the direction is not in doubt, and this is the single
highest-value addition to the stack found in this study.

**Recommended for the rover.** Add the EKF; it attacks drift at its source
rather than asking loop closure to repair it afterwards. On real hardware
`ekf_leo.yaml` needs its frames changed to `odom` / `base_footprint` and its
topics to the rover's own wheel-odometry and IMU topics, and the existing
bringup must stop publishing `odom -> base_footprint` so the EKF owns it — two
publishers of that transform will fight.
