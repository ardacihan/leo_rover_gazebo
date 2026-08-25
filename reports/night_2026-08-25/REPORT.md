# Night 2026-08-25 — distributed shared mapping, merged with or without markers

**Status: DRAFT — being filled in as runs finish. See PROGRESS.md for the tick ledger.**

Branch `feat/multi-robot-integration`. All numbers measured this night unless
marked otherwise; n=1 flagged wherever it is n=1.

## Executive summary

The mission was: two rovers that know nothing about each other build maps,
merge them **with or without markers**, and each uses the other's coverage —
distributed, and ready for the lab at 9am. As of this report:

- **Marker-free merging works, live.** A new global matcher (bidirectional
  FFT search → 120 candidate modes → triage → full-resolution polish →
  margin abstention) scores 6/7 recorded pairs within 0.5 m/10° with **zero
  confident-wrong** (baseline: 0/10, all flips), and locked in **three live
  sim runs with markers and cameras off: 0.45, 0.24 and 0.18 m** — the last
  in the depot, the world where the old matcher flipped 4 of 4 times. When
  it cannot be sure, it abstains and says why; abstention-until-markers is
  the designed production behaviour.
- **The merged map now changes behaviour**: unknown cells the peer already
  covered stop generating frontiers (mask activated live, up to 31,619
  cells), with a seamless fall-back when the merge is absent or stale.
- **It is distributed**: each rover runs its own aligner+merger on the
  peer's map and publishes its own merged map in its own frame. In the
  partition run both rovers locked independently, their merges agreed to
  0.28 m/0.6°, and the merged maps stayed alive after the central node was
  killed.
- **The real launch path is namespaced** behind defaults that leave the
  field-validated single-rover stack byte-for-byte unchanged, and was
  brought up twice (rob-style namespaces) against the simulator through
  the same launch file the rovers will run.
- Compute: local RTX 4060 Ti (D3D12) for interactive runs plus the Viper
  Slurm cluster (apptainer port of the exact sim image, llvmpipe at RTF
  ~0.95) for parallel A/B measurement runs.

Everything below is the evidence; the limitations section is the part to
read before the demo.

## Phase 0 — orientation

- Build: 17 packages, green (`colcon build --symlink-install`).
- Tests, honest version: the goal doc's "61/61" matches no current subset.
  Actual: leo_rover_exploration + leo_rover_real_bringup **78/78 green**
  (later 90/90 with tonight's 12 new tests); leo_nav2_exploration 72/85 —
  **13 pre-existing contract-test failures** (nav2 polygon footprint /
  state-lattice, costmap doorway resolution, DWB footprint, [real] sensor
  topics + slam contracts, operator docs). These predate tonight and belong
  to the nav2-bundle work; the `[real]` ones are relevant to the lab and are
  listed in the limitations.
- Benchmark harness: `_baseline_matcher` reproduces **0/10 with 90/180° flips
  on every depot and office pair** — harness, maps and ground truth confirmed.
- GPU: `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)` (gpu_check.txt of
  the first night run; even camera-off runs render the lidar on D3D12).

## Phase 1 — marker-free merging (offline benchmark + live confirmation)

### The method (scripts/marker_free_matcher.py = multi_robot_shared_mapping/marker_free_matching.py)

1. **Global FFT search, both directions.** Per 2° yaw step, rasterize one
   map's walls+free space at 0.25 m and score every translation at once by
   FFT cross-correlation. Score on the overlap region only: wall-hit reward,
   penalties for walls-on-free and free-on-walls, normalized by how much of
   the moving map lands in the fixed map's known space; hypotheses below a
   25% overlap floor are invalid. The search runs map2→map1 AND map1→map2
   (inverted and pooled): with asymmetric coverage the true hypothesis dies
   at the overlap gate in one direction and passes trivially in the other.
2. **8 translation peaks per yaw** (NMS 2 m), pooled and deduped to ≤120
   candidate modes. 4 peaks was measurably too few: an office is a periodic
   room grid and the false room-shift placements filled every slot before
   the true mode got one.
3. **Triage → full polish.** The coarse score only proposes (measured: it
   ranked the true mode below 30th); the polished wall-overlap decides.
   Every mode gets a ~5 ms single-stage polish; the top 8 get the full
   3-stage coarse-to-fine polish (±1.05 m/±4° stage-1 window — must exceed
   coarse-cell error plus real inter-map drift, measured ~1.9 m).
   Mode quality = max(forward hit, reverse hit), each all-points-normalized
   (overlap-normalization was tried and is the hide-in-unknown-space trap:
   tiny-fragment hypotheses score 0.8+ and flood the list).
4. **Abstention.** Commit only if best quality ≥ 0.80 AND the best distinct
   runner-up (>12° or >2 m away) trails by ≥ 25%. Measured commit margins:
   true modes 0.87–0.99 vs best wrong mode ≤ 0.6–0.7.

### Benchmark (10 recorded pairs, exact spawn-offset ground truth)

| | baseline | final matcher |
|---|---|---|
| within 0.5 m / 10° | 0/10 | **6/7 attempted** |
| confident-wrong (>1 m or >15°, no abstention) | 10/10 | **0** |
| abstained | 0 | 3 |
| time per pair | ~5 s | 4.0–7.1 s |

Per-pair: depot 0.27–0.41 m / 0.7–2.3°; phase4_office 0.40 m / 0.45°.
The 7th attempt (phase2_depot_coordinated, 0.65 m) is **drift-capped, not
matcher-limited**: the tag+refine pipeline lands on the same transform
(0.63 m from spawn truth) using markers — the maps themselves are warped.
The 2 husarion abstentions are honest (partial-overlap corridor maps); the
office_coordinated abstention is the drift-poisoned pair that would
otherwise commit 1.9 m wrong at hit 0.76 — the 0.80 floor exists for it.

Gate 3 caveat, honestly: the two largest office pairs take 6.5–7.1 s in the
container, over the 5 s target; the live aligner runs every ~15 s, so it
runs live regardless.

Gate 4 (markers off entirely): the matcher never sees tag data by
construction; the live confirmation runs used `SKIP_ARUCO=1` (no detector
nodes launched, cameras off) and `alignment_mode:=markerfree`.

### Live confirmation (sim, office_world, markers off)

- Run 1 (pre-fix): self-terminated 2/2, media clean; aligner abstained 64/64
  cycles, zero commits — exposed three wiring bugs (see below) and the
  asymmetric-coverage blind spot (leo1 mapped 2 rooms, leo2 the whole
  office; the visibly-correct merge scored 0.295 under forward-only
  normalization while truth-seeded polish reached q=0.987).
- Bugs found live, each fixed and committed separately: (1) the TF bridge's
  `require_tag_evidence` gate made marker-free locking impossible — launch
  now drops it in markerfree mode only; (2) `shared_map_merger`'s mode gate
  never subscribed the accepted transform in markerfree mode — it could
  lock and still never merge; (3) forward-only scoring (fixed by the
  bidirectional/triage architecture above).
- Run 5 (full chain, `phase1_markerfree_office_run5`): **PASS.** 34 honest
  abstentions while the maps were disjoint (best hit 0.36–0.76, always
  under the floor), then **lock at t≈645 s: (11.30, −9.66, −178.7°) =
  0.45 m / 1.3° from ground truth, confidence 0.80→0.84**, held to the end
  of the run. Both explorers self-terminated (t=785 s), zero failed goals.
  Media: the merged map is the complete office with single walls — no seam,
  no doubling, no speckle (`merged_map.png`). In the same run the Phase 2
  mask activated after the lock: leo2 suppressed 31,619 unknown cells as
  peer-covered. n=1.

## Phase 2 — the merged map changes what the rovers do

Implementation: the explorer subscribes the merged map (VOLATILE QoS — the
merger publishes VOLATILE and a TRANSIENT_LOCAL request would silently never
match). Unknown cells of the own map that a fresh merged map already knows
are treated as covered BEFORE frontier detection — applied to the unknown
mask, not the frontier cells (those are free in the merged map by
construction; masking them erases every frontier). Stale map (>20 s),
missing topic or missing alignment TF → seamless own-map fallback. The
independent condition never receives the topic, so the baseline is
byte-identical. 6 unit tests.

Live evidence (coordinated runs, all marker-free, `SKIP_ARUCO=1`):

| run | lock err | conf | abstains before | mask cells (leo2) |
|---|---|---|---|---|
| office local (run5) | 0.45 m / 1.3° | 0.84 | 34 | 31,619 |
| office Viper (phase2v) | 0.24 m / 0.1° | 0.94 | 37 | 17,635 |
| depot Viper (phase2v) | **0.18 m / 0.0°** | 0.94 | 11 | — |

The depot lock matters most: it is the world where the old grid matcher
flipped 180° on 4 of 4 runs. Three live locks, zero confident-wrong.

Coordinated metrics (metrics script, world-bounds-clipped, truth-transform
duplicated coverage): office coordinated t90=495 s, dup 287 m², 53 goals /
1 failed; depot coordinated t90=585 s, dup 162 m², 41 / 2.

A/B, coordinated vs independent, one seed each (all on Viper, same knobs,
markerfree, no cameras; the first two baselines were broken by a runner bug
— empty `shared_map_topic:=` killed the explorer launch — fixed and rerun):

| map | condition | final m² | t90 s | **dup m²** | goals | failed |
|---|---|---|---|---|---|---|
| office | coordinated | 181.2 | 495 | **287.1** | 53 | 1 |
| office | independent | 179.8 | 555 | **354.9** | 67 | 1 |
| depot | coordinated | 132.8 | 585 | **162.2** | 41 | 2 |
| depot | independent | 133.7 | 525 | **180.9** | 46 | 4 |

**Gate met on both maps**: coordination cuts duplicated coverage by 19%
(office) and 10% (depot), with the same final area. Office also finishes
faster with fewer goals; depot's baseline reaches t90 a minute earlier —
honest split, consistent with the documented "rendezvous arrives late"
mechanism (the lock lands in the final third, so the mask can only claim
the last minutes; the small depot leaves it least room). No stalls in any
condition; run5 additionally shows the seamless pre-lock fallback (the
explorers ran fine through 34 abstain cycles before the merge existed).
n=1 per cell.

## Phase 3 — distributed

Per-rover aligner+merger pairs (`distributed_shared_map.launch.py`): each
rover consumes the peer's `/map`, aligns locally (markerfree matcher with
its own floor/margin/jump vetting), merges locally, publishes
`/{self}/shared_map` **in its own map frame** — which makes the frontier
mask TF-free (the explorer reads the grid's `header.frame_id` and uses the
identity offset for its own frame). The only remaining central piece is the
optional coordination-TF bridge; the merge path has no shared dependency.
Only `/leo{i}/map` crosses the network (d241087: our own DDS traffic starved
the rover firmware).

Partition run (`phase3_partition_office`, `DISTRIBUTED=1 PARTITION_AT_MIN=13`,
office, markerfree, no cameras), n=1:

- **Both per-rover aligners locked independently, no markers:** leo1's
  estimate of leo2 (11.26, −9.72, 180.4°) = 0.38 m/0.4° from truth at
  confidence 0.91; leo2's estimate of leo1 (11.11, −9.87, −181.0°) =
  0.17 m/1.0° from truth at 0.92. Composing the two estimates gives
  (0.16, 0.23, −0.6°) against identity — **the rovers' merged maps agree to
  ~0.28 m / 0.6°, within the alignment error.** Media: each rover's
  `shared_map` is the complete office, single walls, and they are exact
  180°-rotations of each other (own frames), furniture matching.
- The central bridge was killed at minute 13; both `/leo{i}/shared_map`
  topics were alive and served the map saver **~40 s after the central node
  died**. Honest caveat: the explorers finished 4 s after the kill, so
  "keeps exploring after the partition" is only briefly demonstrated live —
  it is architecturally guaranteed (the per-rover mergers never subscribe
  to the bridge), but the run does not show minutes of post-partition
  exploration. Both explorers self-terminated; the run was clean.

## Phase 4 — lab readiness

- `real_mapping.launch.py` namespaced behind `robot_ns` (default empty =
  byte-for-byte the 2026-08-20 field configuration). With `robot_ns:=rob_a`:
  the whole cmd_vel chain, scan chain, and ArUco topics gain the prefix,
  slam_toolbox keeps its load-bearing absolute-`/map` remap (to
  `/rob_a/map`), frames become `rob_a/map|odom|base_footprint`, `/tf` stays
  global. `use_ekf:=true` together with `robot_ns` fails loudly (the EKF
  belongs to the rover's own bringup in multi-rover mode).
- **Rehearsal: PASSED** (`phase4_rehearsal`, 4 attempts, each earlier
  failure a real find): both real_mapping stacks up under leo1/leo2
  namespaces against the sim, `/leo{i}/map` published by the real slam,
  prefixed TF `map->base_link` resolving on both, and a teleop command
  entering the TOP of the safety chain drove each rover ~0.8 m
  (leo1 0.85 m, leo2 0.79 m) through smoother → guard → collision monitor.
  The rehearsal caught two bugs that WOULD have burned lab time at 9am:
  (a) bare-node-name parameter yamls silently do not load under a
  namespace in Humble — the collision monitor aborted lifecycle bringup on
  a missing polygon type and slam ran on defaults; params are now loaded
  and passed inline in namespaced mode; (b) the scan box filter's
  `base_footprint` frame must be prefixed — unprefixed, every scan was
  dropped and the guard/monitor/slam all starved with the chain looking
  healthy. (One harness-side note: the final verdict comparator used
  python3 on a host that has only python; the corrected summary carries
  the amendment and the raw numbers.)
- LAB_SESSION.md: updated (bring-up, marker-free fallback with live
  numbers, distributed option, clock-sync first step, stall table).
- Clock sync between machines: NOT rehearsable in sim (single clock);
  documented as the first bring-up step with the exact commands.

## Viper (remote Slurm cluster, 8 APU slots)

Ported and used tonight: the exact `leo_rover_humble:bundle` docker image
runs under apptainer on the MI300A nodes (llvmpipe software rendering —
`--contain` hides /dev/dri, which otherwise crashes OGRE on the
permission-denied DRM devices; RTF ~0.95 lidar-only, cameras untested).
The offline benchmark reproduces 0/10 there end-to-end. Phase 2 A/B
measurement runs execute there in parallel with the local GPU runs.
Details and copy-pasteable commands: `viper_recon.md`.

## Honest limitations

1. **Strict Phase 1 pass count is 6/10, not 7/10.** The 7th attempted pair
   (0.65 m) is drift-capped — the tag pipeline lands on the same transform —
   and the two husarion pairs abstain on genuinely partial overlap. The
   gate's hard clause (zero confident-wrong) is met everywhere, including
   three live runs.
2. **Matcher speed**: 4.0–7.1 s/pair in-container; the two largest office
   pairs exceed the 5 s target by ~2 s. The live aligner cycles every
   ~15 s, so this does not block live use.
3. **The lock arrives late** (~10–11 min in a ~13 min exploration run), so
   the mask can only influence the final minutes; coordination gains over
   independent are correspondingly modest. This is the known "rendezvous
   arrives late" behaviour, now with marker-free numbers attached.
4. **Post-partition exploration was demonstrated for only ~4 s** (the
   explorers finished right after the kill). Merge-path independence from
   the central node is by construction (no subscription), and both merged
   maps served the saver 40 s after the kill — but a long post-partition
   exploration was not observed.
5. **All of tonight is simulation.** The namespaced real launch path loads
   and runs against the sim; none of it has touched hardware. The 13
   pre-existing `leo_nav2_exploration` contract-test failures include
   `[real]`-profile sensor/slam contracts — recheck against the rovers
   before trusting those configs.
6. **Clock sync between machines has zero overnight coverage** (single sim
   clock); it is called out as the first bring-up step in LAB_SESSION.md.
7. **Loader quirk**: the benchmark's map loader reads the saved-map unknown
   pixel as free (yaml free_thresh 0.25 vs pixel 205). Harmless for wall
   matching (thresholds were calibrated with it, and the live aligner
   consumes proper OccupancyGrids), but any future *area* metric must use
   the strict loader in `phase2_metrics.py`.
8. **n=1 or n=3 everywhere.** Depot's own history shows 0.19–2.25 m
   variance across identical runs; do not over-read any single number
   above.
9. Viper runs used llvmpipe software rendering, lidar-only; **cameras (and
   therefore ArUco) were never tested on Viper** — marker runs stay local.

## Deliverables checklist

- [x] PROGRESS.md ledger (appended every tick)
- [x] Phase 1: benchmark before (0/10) and after (6/7 + 3 abstain, 0 wrong),
      method written down, live confirmation run committed with media
- [x] Phase 2: per-map coordinated vs independent with duplicated-coverage
      (gate met on both maps)
- [x] Phase 3: partition run (independent locks agreeing to 0.28 m/0.6°,
      merged maps alive after the central kill)
- [x] Phase 4: rehearsal PASSED (2 real bugs found and fixed on the way),
      LAB_SESSION.md updated
- [x] Every merged map as .pgm/.yaml AND rendered .png
- [x] Dashboard: reports/night_2026-08-25/dashboard.html (7 runs, films)
- [x] Committed and pushed to feat/multi-robot-integration, files staged
      explicitly throughout (~15 commits, 520fd6c … end of night)
