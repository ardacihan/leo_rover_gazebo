# Multi-robot integration — running ledger

**Branch:** `feat/multi-robot-integration` (checked out at `513faba`)
**Date:** 2026-08-23
**Goal doc:** `OVERNIGHT_GOAL_MULTIROBOT.md`

The ledger is the source of truth for where this session is. Read it first on
every tick. Never restart a phase marked COMPLETE.

## Plan

| Phase | What | Status |
|---|---|---|
| 0 | Build, unit tests, 3-min two-rover smoke on depot_world, GPU confirmed | IN PROGRESS |
| 1 | One honest integrated run on husarion_office: no gt odom, no identity map offsets, tag alignment | NOT STARTED |
| 2 | 3 maps x 1 seed (depot_world, office_world, husarion_office), coordinated first | NOT STARTED |
| 3 | Lab-ready namespacing + DDS + marker card + office_world rehearsal run, LAB_SESSION.md | NOT STARTED |

Hard rules carried from the goal doc:
- 25 min wall-clock cap per sim run. Runs are sequential.
- Media review with the Read tool before believing any number.
- One seed per map. Coordinated losing to uncoordinated is a RESULT.

## Ledger

### 2026-08-23 — tick 1 — Phase 0 (build + survey), nothing running

**Build:** `feat/multi-robot-integration` @ `513faba` checked out. Full
`colcon build --symlink-install` in a throwaway `leo_build` container: **17
packages, 45 s, zero errors or warnings.** `install/` now carries
`multi_robot_shared_mapping`, `multirobot_map_merge`, `leo_rover_real_bringup`
and `leo_rover_semantic_vision`, none of which were built before. The textual
merge did not break the build.

**Unit tests:** `multi_robot_shared_mapping/tests/` + `leo_rover_exploration/test/`
→ **61 passed in 1.43 s**. Gate item 1 of Phase 0 is green.

**Findings that change the plan (all from reading/measuring, no sim yet):**

1. **The goal doc's `marker_length := 0.1333` is wrong, and so is the
   detector's 0.15 default — for the worlds that matter it is 0.20.**
   Measured the actual textures rather than trusting the derivation: in
   `models/aruco_N/materials/textures/aruco_N.png` the black square fills
   **100%** of the image (600x600 for id 0, 200x200 for 1-15) — there is no
   quiet zone in the texture at all, so the black square *is* the 0.20 m
   plate. The doc's 4/6 figure assumes padding that is not there and would put
   every tag pose **33% short** along the view ray; the 0.15 default, 25%
   short. Corroborated independently: `scripts/aruco_test_wsl.sh:70` already
   defaults `ARUCO_LEN=0.20` for the sim, so an earlier session reached the
   same number empirically. **Decision: `marker_length := 0.20`.**

2. **office_world already has correct, real ArUco markers baked into the
   world** (`office_world.sdf:216+`): white backing board for the quiet zone,
   a 0.20 m textured black square, `aruco_N_bare.png`. Nothing to do for
   office_world. Its plate is `<box>0.01 0.20 0.20</box>` — thin along **x** —
   so the face normal is +x at yaw 0 and the `mock_markers_*.yaml` convention
   (`normal = (cos yaw, sin yaw)`, documented as "yaw = outward wall normal")
   is exactly right. Verified against every marker comment in the file
   (m3 north wall yaw=-pi/2 faces south into N1, m1 west wall yaw=0 faces east
   into the corridor, etc.).

3. **The standalone `models/aruco_N/` plates use the opposite convention and
   have no quiet zone.** Their box is `0.20 0.001 0.20` rotated pitch 90°, so
   the thin axis is **y** and the normal is ±y at yaw 0 — spawn them with the
   yaml yaw and every marker sits *edge-on* to the room. Combined with the
   missing quiet zone, the `two_robots.launch.py` marker path could not have
   worked. I am not using it: I standardize on the office_world geometry via
   an inline SDF, one convention everywhere. (Patched the 16 models with a
   backing board, then reverted it — shipping a second, untested convention is
   worse than one good one. The defect is recorded here for the report.)

4. **depot_world's 4 markers are still magenta placeholders**
   (`<box>0.02 0.15 0.15</box>`, flat magenta, no texture) — invisible to a
   real detector. Same +x-normal convention as office_world. They need
   upgrading to the office_world pattern before depot can be a Phase 2 map.

5. **husarion_office has no markers in the world at all** and, being
   mesh-collision, cannot be rasterized by `scripts/world_ground_truth.py`
   (it recovers only pillars — see `worlds/husarion_office.png`). Coverage
   there must use the doc's manual bounds `-4 27 -15 4`; the spawn-clearance
   check I built is only meaningful for the authored `leo_rover_gazebo` worlds.

6. **Coordination is wired to frames that will not exist in this run.**
   `frontier_explorer._get_common_offset` (`:755`) looks up `map -> leo{i}/map`
   and `_peer_xy` (`:774`) looks up `leo{i}/map -> {peer}/base_link`. Both only
   resolve because `map_merge_leo.launch.py` publishes identity static TFs —
   the exact cheat Phase 1 forbids. Without a replacement, "coordinated" mode
   silently degrades to independent and the night measures nothing. Needs a
   node that republishes the *estimated* leo1/map -> leo2/map once alignment is
   confident. This is the real content of Phase 1 item 7, more than the
   `_get_peer_offset` caching bug the doc names.

**Spawn poses picked** (rendered + clearance-checked, `worlds/*.png`):
depot leo1 `(0, 4.5)` north-central, leo2 `(3, -4.5)` SE room — different
rooms, both free. office leo1 `(-8, 5)` was **blocked** by a prop; moved to
`(-7, 5)` in room N1, leo2 `(4, -5)` in room S2. husarion keeps the authored
leo1 `(0,0)` / leo2 `(2.36, -11.27)`.

**Next:** wire the marker spawning + two ArUco detectors + the alignment-TF
bridge, then the 3-min smoke on depot. No sim has been run yet this session.

### 2026-08-23 22:58–23:05 — tick 2 — Phase 0 SMOKE on depot_world: **PASS**

`scripts/smoke_multirobot.sh depot_world` — sim + realistic odom + SLAM x2 +
ArUco x2 + alignment + Nav2, both rovers spun in place to sweep for markers,
then asserted. Artifacts in `smoke_depot/`.

| Assertion | Result |
|---|---|
| `/leo1/map` publisher count | **1** (the slam_multi remap holds) |
| `/leo2/map` publisher count | **1** |
| `/leo1/tag_detections` type | `visualization_msgs/msg/MarkerArray` |
| `/leo2/tag_detections` type | `visualization_msgs/msg/MarkerArray` |
| leo1 confirmed markers | **2** (ids 1, 3) |
| leo2 confirmed markers | **2** (ids 2, 4) |
| `compute_path_to_pose` servers | **2** |
| alignment nodes up | `shared_map_merger`, `tag_based_map_aligner`, `map_based_aligner`, `alignment_tf_bridge` |
| ground-truth TF diverted off `/tf` | `/leo1/tf_ground_truth`, `/leo2/tf_ground_truth` present |
| `leo1/odom -> leo1/base_link` | published by `sim_realism_odom.py`, not Gazebo |

**GPU: confirmed on the GPU.** `/root/.ignition/rendering/ogre2.log` says
`GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)`, `GL_VERSION = 4.2 Mesa`.
Not llvmpipe. The patched Ogre + surfaceless EGL path in `sim_gpu_wsl.sh`
works, so the docker route is fine and native WSL is not needed. (`nvidia-smi
--query-compute-apps` on the Windows host cannot see WSL/container processes —
it reports "Insufficient Permissions" — so the Ogre log is the check that
decides, and both scripts now read it from its absolute path with a retry
instead of grepping `~` too early.)

**`marker_length = 0.20` confirmed empirically, and the doc's 0.1333 refuted.**
From `aruco_samples_leo1.csv`, marker 1 (true `(0.0, 6.88)`, rover looking
north so *y is the view ray*) was detected at y ≈ 6.78–6.81 from 2.11 m: an
along-ray error of **0.08 m, i.e. 3.8% short**. `marker_length = 0.15` would
put it 25% short (0.53 m) and 0.1333 would put it 33% short (0.70 m). Neither
is what was measured. leo2/marker 2 shows the same 0.09 m along-ray error.

**Two things the smoke found that change how Phase 1 should be read:**

1. **Tag landmark positions inherit the rover's SLAM error, and that is now
   the dominant term.** Markers 1 and 2 land ~0.45 m out *laterally* (leo1
   -0.45 m in x, leo2 +0.45 m in x — opposite signs, i.e. independent per-rover
   drift, not a systematic detector bias). With ground-truth odom this would be
   ~0. It is the honest number and it sets the floor on how well tags can
   align two maps. It is inside `landmark_gate_distance` (1.0 m), so good later
   observations will not be gated out as outliers.
2. **Distant detections under rotation are unusable.** Marker 3 at ~5.0 m was
   seen 4 times with a steady camera-frame observation (cam ≈ 4.5–4.9 m ahead,
   1.0–2.4 m left) but its *map* position wandered over 8 m across 31 s —
   the rover's yaw estimate was spinning, not the marker. 25 px of marker at
   5 m plus a moving pose estimate is noise. **Decision: `max_range` 6.0 → 4.5
   for the runs.** Close detections (2.1 m, 55–64 px, reproj < 0.2 px) were
   rock solid.

   The spin itself caused most of this: 0.6 rad/s for 55 s is ~5 revolutions,
   and the realistic odometry's 12% yaw error over that is huge. The smoke now
   spins at 0.30 rad/s; the real runs drive, they do not pirouette.

**Pre-rendezvous behaviour works as designed.** With both rovers confined to
their own rooms the aligner logged `landmarks leo1=[1,3] leo2=[2,4] common=[]`
and `alignment_tf_bridge` stayed silent — "waiting for an alignment estimate".
No transform is published, no bogus shared frame, and the explorers would fall
through to independent allocation. That is the correct degrade path and it is
now observed rather than assumed.

**Minor, recorded not fixed:** every landmark's `map_z` is ~0.195 m low
(marker true z = 0.30, reported 0.105) because the TF chain runs
`odom -> base_link` and skips the URDF's `base_footprint -> base_link`
z = 0.19783. It is a constant z bias, identical for both rovers, and the
alignment is 2D — so it cannot affect this night's result. Worth fixing before
anything uses tag height.

**Process note:** the depot smoke's own run stalled partway through its
assertion list because I edited `smoke_multirobot.sh` *while bash was still
reading it* — bash reads a script incrementally by byte offset, so inserting
lines mid-run corrupts what it executes next. The remaining assertions were
re-run by hand against the still-live container (results above). Do not edit a
running shell script.

**Next:** short smoke on husarion_office to check the ported marker yaws
before committing a 25-minute Phase 1 run there — husarion's markers are the
only ones spawned from the converted table rather than baked into the world,
and a 90-degree error would mean no detections at all.

### 2026-08-23 23:05–23:10 — tick 3 — husarion marker smoke, then **PHASE 1 LAUNCHED**

**husarion marker smoke (`smoke_husarion/`): the ported yaws are correct.**
leo1 confirmed **ArUco 2 at map (1.77, 0.00)** against a ground truth of
**(1.79, 0.0)** — a **2 cm error** at 1.53 m. leo1 spawns at the origin so its
map frame is the world frame, which makes that a direct check of the whole
chain: the +pi/2 yaw conversion, `marker_length = 0.20`,
`frame_is_optical = false`, and the double-sided inline plate SDF. Under the
*original* table's yaw the plate normal would have been -y and the marker
edge-on from the origin — it would never have detected. The conversion was
necessary, and it is now verified rather than argued.

**leo2 saw nothing at all — 296 frames, zero detections — and the reason is
marker placement, not code.** From leo2's spawn `(2.36, -11.27)` its two
nearest markers are badly oblique:

| marker | distance | angle off normal | apparent width | pixels |
|---|---|---|---|---|
| 7 `(4.01, -7.54)` | 4.1 m | 59° | 0.10 m | ~14 px |
| 0 `(6.51, -11.97)` | 4.2 m | 82° | 0.03 m | ~4 px |

`min_marker_px` is 18, so both are below the floor. **Fix: marker 7 re-aimed to
yaw = pi/2 (north/south).** The route between the two spawns runs north-south
around x ≈ 1.5–4, and the plates are textured on *both* faces, so a north/south
normal is face-on to leo1 coming from the north *and* leo2 coming from the
south. Markers 6, 7 and 8 now all sit normal to that shared corridor — three
tags both rovers can read, against a `min_tags` of 2. Nothing else was moved.

Deciding to texture both plate faces (rather than model a one-sided printed
marker) is what makes this work: a single-sided tag on that corridor would be
readable by exactly one of the two rovers, which is the one thing tag alignment
cannot use. Noted as a sim-only liberty — Phase 3 has to say what the lab does
instead.

**`max_range` 6.0 → 4.5** in the run harness, on the smoke evidence: a marker
at 5.0–5.5 m (23–25 px) produced map positions that wandered over 8 m while its
camera-frame observation stayed steady, because the rover's yaw estimate was
moving. Landmarks are persistent and anchor the outlier gate, so a badly placed
one is worse than a missing one.

---

**PHASE 1 RUNNING — started 23:10:47, cap 25 min, ends 23:35 at the latest.**

`scripts/auto_multirobot_run.sh coordinated husarion_office \
  reports/multirobot_2026-08-23/phase1_husarion_coordinated 25`

The no-cheating proof, from `cmdlines.txt` (written by the run itself):

- `GT_ODOM_TF=false` — Gazebo's true pose is diverted to `/leo{i}/tf_ground_truth`;
  `sim_realism_odom.py` owns `odom -> base_link`, seeds 1 and 2.
- **`map_merge_leo.launch.py` is not launched at all** — no identity statics.
- `alignment_mode:=hybrid`, `enable_tag_alignment:=true`,
  `enable_map_alignment:=true` — **not `fixed`**.
- `compare_to_ground_truth:=true ground_truth_x:=2.36 ground_truth_y:=-11.27
  ground_truth_yaw:=0.0` — scoring only; the offset comes from the same
  `spawn_poses.py` table the sim spawned from, so the measured error cannot
  drift from the simulated geometry.
- `common_frame:=leo1/map` on the explorers, fed by `alignment_tf_bridge`,
  which publishes nothing until confidence ≥ 0.5.

**Check at 23:14 (≈4 min in):** container up; `/leo{1,2}/map` publisher count
1 each; GPU `D3D12 (NVIDIA GeForce RTX 4060 Ti)`; traj.csv 22 rows and growing;
both explorers assigning goals (leo1 → `(0.83, -2.46)`, leo2 → `(2.09, -10.79)`
— *different rooms*, as intended); alignment `t=105s no estimate yet
tags=0/0 common=0`, which is correct this early.

**One gap found and closed mid-run:** `/shared_map` does not publish until
alignment locks, so `coverage.log` would have stayed empty for the whole run
and a failed alignment would also have cost the coverage curve. Started
per-robot coverage monitors (`coverage_leo1.log`, `coverage_leo2.log`, same
`-4 27 -15 4` clip) by `docker exec` — read-only and additive, it does not
touch the run. At t=120 s: leo1 12.2 m², leo2 3.2 m². Folding this into the
harness for Phase 2.

### 2026-08-23 23:10–23:45 — tick 4 — **PHASE 1 RUN 1: FAILED (leo2 wedged). Media reviewed.**

Ran to the 25-minute cap. Artifacts + all 5 figures in
`phase1_husarion_coordinated/`. **Gate: 1 of 4.**

| Gate item | Verdict |
|---|---|
| 1. Both rovers explored, run ended on its own | **FAIL** — leo2 never explored; run hit the cap with `finished=1/2` |
| 2. `/shared_map` published, transform within 0.5 m / 10° | **FAIL** — `/shared_map` had a publisher but never published a message; no transform was ever produced |
| 3. Media review | **PARTIAL** — leo1's map is clean; leo2's is a stub |
| 4. No gt-odom, no identity offsets, no fixed alignment | **PASS** — see `cmdlines.txt` |

**What the pictures actually show.**

`leo1_map.png` — **clean.** Single-pixel walls throughout, straight and
unbroken; the office reads as an office (west corridor, the long north-south
corridor at x≈8, the pillar/desk cluster around x 5–7, y −7 to −9, the east
rooms with their radiator fins along y≈−4.5). **No doubled walls, no ghosting,
no seam, no speckle fields in free space.** All geometry sits inside the world
footprint (x −0.7…13.3, y −12.2…0.9). 79 222 known cells, 3 694 occupied. This
is a genuinely good map built with *corrupted* wheel odometry and no
ground-truth prior — the thing the realism harness was supposed to make hard.

`leo2_map.png` — **a stub.** One small room outline around x −0.6…4,
y −13.5…−8.5 and nothing else, plus isolated speckle points trailing
south-east to y≈−16.2, which is **outside the world's southern bound (−15)**.
By the doc's own table that is drift, and it is why the coverage number is
clipped: unclipped it would flatter a rover that mapped almost nothing.

`traj_overlay.png` — the headline, and it is unambiguous. leo1 (blue, 429
samples) runs from the origin down the west corridor, across the middle, east
to x≈12, and south to y≈−10, passing close to markers 2, 6, 8, 7, 9, 1 and 0.
leo2 (orange, 313 samples) is **a single blob 1 m across at (1.4, −11)** — it
never left its spawn. Not oscillation, not a livelock: it simply never got out.

`coverage.png` — leo1 climbs to **107.4 m²**; leo2 **flatlines at 16.7 m² after
2.2 minutes** and is a horizontal line for the remaining 11.

`frames_leo2/raw004_t307.png` — the diagnosis, in one frame: **a blank wall
filling the entire image from about half a metre away.** That is what leo2's
camera saw for 300+ frames, which is exactly why it confirmed zero markers.
For contrast `frames_leo1/raw003_t246.png` shows a full office scene — stairs,
desk, chair, cabinet — so the camera and GPU rendering are fine.

**Root cause: the authored leo2 spawn, not the integration.** `(2.36, −11.27,
z=0.05)` — inherited from `two_robots.launch.py` — puts the rover against a
wall in a confined pocket. True pose after 25 min: `(1.36, −10.45)`, i.e. 1.3 m
of travel. Everything downstream follows: no marker sightings → no common
landmarks → no tag transform → `alignment_tf_bridge` correctly stays silent →
no `/shared_map` → no merged coverage.

**The integration itself worked.** leo1 accumulated **6 persistent landmarks
[0, 2, 5, 6, 8, 9]** through the real detector, and the one landmark I could
check against truth was tag 2 at `(1.7751, 0.0009)` versus `(1.79, 0.0)` — a
**1.5 cm error**, confidence 0.984 over 220 observations. The chain from camera
to persistent landmark map is sound; it had one participant instead of two.

**SLAM drift, measured** (SLAM estimate vs Gazebo truth, at ~12 min):
leo1 1.57 m, leo2 0.57 m. Honest numbers for realistic odometry, and the
reason tag landmarks carry ~0.5 m of error.

**Re-run is justified by the doc's own list** — "a rover wedged in the first
3 minutes and never recovered". This is not chasing significance; it is a
broken run.

**The fix is evidence-based, not another guess.** husarion_office has mesh
collisions, so the rasterizer cannot vet a spawn there — which is how the bad
pose survived. New tool `scripts/pick_spawn_from_map.py` scores candidate
spawns against **a map a rover actually built**, treating unknown as blocked.
Its top picks by raw clearance were all at y≈−12.57, *south of the building's
own south wall* — free-looking only because nothing ever observed them, a nice
demonstration that clearance on a SLAM map is not the same as free space. So
the pose was instead chosen from **leo1's own trajectory**: every sample is a
place a rover physically fitted. leo2 now spawns at **(9.60, −9.95, yaw π)** —
clearance 0.56 m, **13.8 m from leo1's spawn**, opposite corner of the
building, facing west along the southern corridor. Ground-truth offset
recomputed automatically to (9.60, −9.95, π).

Also folded into the harness before the re-run, both from this run's failures:
per-rover coverage *and* per-rover own-frame trajectory recorders (the
shared-frame ones are blind until alignment locks — that cost the entire
coverage curve and every leo2 trajectory sample here), and the renderer now
falls back to them and labels the unaligned track as such.

**Process note:** the harness process was killed partway through teardown by a
timed-out monitoring command in my own session, so it never saved maps itself.
The container was still healthy, so the maps were saved by hand from the live
sim and nothing was lost. Re-run launches with `setsid` so it cannot be caught
by that again.

### 2026-08-24 00:15 — tick 5 — **PHASE 1 RUN 2: both rovers explored, alignment converged, merged map FAILS the eye test**

`phase1_husarion_coordinated_run2/`, full 6-figure media set. Spawn fix worked;
a deeper problem with the *test itself* surfaced.

**The spawn fix worked.** leo2 went from 16.7 m² (wedged) to a full traverse of
the building. Final clipped coverage: **leo1 84 m², leo2 69 m², merged
130.9 m²** — versus run 1's 107 / 17 / none. Both rovers detected markers
(leo1 6 landmarks, leo2 3) and reached **3 common landmarks**.

**Alignment converged — but I was scoring it against the wrong ground truth.**
The run reported `err=13.92 m` all the way through. That number is wrong, and
chasing it is how a night gets wasted, so: leo2's landmark map puts tag 8 at
`(6.15, -4.38)` and tag 0 at `(6.29, -12.12)` against world truth
`(6.28, -4.41)` and `(6.51, -11.97)`. **leo2/map is the world frame**, not
leo2's spawn frame — so the true leo2→leo1 transform is *identity*, and my
`relative_offset()` (spawn difference) was the wrong target.

Root cause, and it is a genuine leak: `scripts/sim_realism_odom.py:111` seeded
its integration on the **true world pose** ("so map and odom share an origin" —
sensible for one robot). With two robots that hands both SLAM maps the same
world frame, so the transform this whole night exists to recover is identity by
construction and the rovers secretly share a frame from the first scan. This is
the world-anchored-odom trap from 2026-07-13, resurfacing in a new place.

Scored against the **correct** target (identity), the pipeline did well:

| | common tags | tag estimate | accepted (fused) estimate |
|---|---|---|---|
| first lock, t=320 s | 1 | 0.297 m / 3.30° (conf 0.18) | 0.892 m / 6.25° (conf 0.51) |
| final, t=865 s | 3 | 0.426 m / 3.16° (conf 0.76) | **0.123 m / 1.74°** (conf 0.61) |

The accepted transform converged 0.89 m → **0.12 m and 1.7°** as common
landmarks went 1 → 3, comfortably inside the gate's 0.5 m / 10°. That is real
recovery — no node was ever told the offset — but it was an *easy* recovery,
because the answer happened to be identity. **Fixed:** `sim_realism_odom.py`
gains `zero_origin`, and the harness now passes `zero_origin:=true`, so each
rover's odometry starts at (0,0,0) the way real wheel odometry does, each map
is anchored on its own rover's start pose, and the true transform becomes the
actual spawn offset. Phase 1 must be re-run under that before its gate means
anything.

**Media review — merged map FAILS.** `merged_map.png` shows **doubled,
offset-parallel walls** through x ≈ 8–9.5, y ≈ −3…−11: the long corridor wall
appears three or four times, offset by roughly 0.2–0.5 m, and the horizontal
wall at y ≈ −7.5 is doubled the same way. By Ground Rule 1 that is a FAILED
merge regardless of the coverage number.

The cause is mostly **leo2's own SLAM drift**, not the alignment. `leo2_map.png`
on its own is visibly worse than leo1's: walls are dashed and broken rather
than solid, and there is speckle out to **y ≈ +5.5, well past the building's
northern edge at y ≈ +1** — the doc's "map geometry outside the world bounds"
signature. Fusing a clean map with a drifted one under a 0.12 m transform still
produces doubled walls, because the two maps genuinely disagree about where the
walls are. `leo1_map.png` (run 1) remains the reference for what good looks
like: single-pixel, unbroken walls.

**Two more bugs found and fixed, both QoS, both silent:**

1. `map_coverage.py` subscribed **TRANSIENT_LOCAL**; `shared_map_merger`
   publishes `/shared_map` **VOLATILE**. A TRANSIENT_LOCAL subscriber is
   incompatible with a VOLATILE publisher, rmw never matches them, and the only
   warning appears on the *publisher* side. The monitor printed "no map yet" for
   an entire run while the merger was publishing happily. Now VOLATILE, which
   matches both. (Same family as the `/wheel_odom` RELIABLE-vs-BEST_EFFORT bug
   from 2026-08-20.)
2. **`nav2_map_server map_saver_cli` cannot save `/shared_map` at all** — it
   only ever subscribes TRANSIENT_LOCAL, so it fails with "Failed to spin map
   subscription". The merged map, the single artifact the two-rover pipeline
   exists to produce, was unsaveable with the stock tool. New
   `scripts/save_map_volatile.py` subscribes VOLATILE and writes the same
   map_server pair.

Also new: `scripts/fuse_maps_offline.py`, which fuses two saved per-robot maps
under any transform in about a second. It rebuilt this run's merged map after
the fact, and it is how alignment gets iterated from here — the doc is emphatic
that re-simulating to test a merge change is a waste, and at 25 min a run it is.

**Process failure, repeated: I edited `auto_multirobot_run.sh` while it was
running**, to add `zero_origin`. Bash reads a script incrementally by byte
offset, so the insertion corrupted what it executed next and teardown died with
a syntax error at line 295 — the same mistake I recorded two ticks ago. The
container survived both times so no data was lost, but the harness must be
copied to a per-run snapshot and executed from that. Doing this before the next
run.

**Phase 1 gate: still not passed.** 2 of 4 (no-cheat proof holds; both rovers
explored). Alignment converged but against an identity target; merged map fails
visually. Next: run the harness from a snapshot, with `zero_origin:=true`, and
judge the gate on that.

### 2026-08-24 00:22–00:45 — tick 6 — **PHASE 1 RUN 3 (honest odom): alignment FAILS on husarion. Gazebo segfaulted at t=550 s.**

First run with `zero_origin:=true`, so each map is anchored on its own rover's
start pose and the true leo2→leo1 transform is the real spawn offset
**(9.60, −9.95, 180°)**, 13.8 m apart. Verified live before trusting it: leo2's
`odom → base_link` read `(0.076, −0.637)` instead of its world pose. Launched
via `scripts/run_snapshot.sh`, so mid-run edits could not corrupt it — that
part worked.

**Alignment result, against a real offset for the first time:**

| | common tags | tag estimate | accepted estimate |
|---|---|---|---|
| t=310 s | 3 | 2.32 m / 96.0° (conf 0.68) | 3.58 m / 65.3° (conf 0.50) |
| t=450 s | 3 | 2.58 m / 110.9° (conf 0.82) | 3.58 m / 65.3° (conf 0.56) |
| final | 3 | **2.58 m / 110.9°** | **3.58 m / 65.3°** |

**Both are far outside the 0.5 m / 10° gate, and confidence is actively
misleading**: the tag estimate reports 0.82 confidence while being 111° wrong.
Run 2's apparent success (0.12 m / 1.7°) was an artefact of the world-frame
leak — the answer was identity, so almost anything near identity scored well.
**This is the honest number, and it is a failure.**

`merge_comparison.png` isolates the cause, and it is the most useful picture of
the night: the *same two maps* fused under the recovered transform (left) and
under the true offset (right). Left, leo2's map is rotated ~65° and lays
diagonal wall fragments straight across the building interior. Right, the
building reads correctly — north room, corridor, pillar cluster, south wall at
y ≈ −12 — as one coherent structure with some wall thickening. **So the maps
were good enough to merge; the transform is what failed.** That splits the
problem cleanly and says where Phase 2 effort belongs.

Coverage at the stall: leo1 100.4 m², leo2 39.6 m², merged 107.8 m² (the
shared-map coverage monitor works now that the QoS is fixed).

**Why the run ended: the Gazebo server segfaulted.** `stall_diagnosis.txt`:

```
[ign-1] D3D12: Removing Device.
[ign-1] Segmentation fault (Address not mapped to object [0x180])
        in /usr/lib/wsl/lib/libd3d12core.so
[ERROR] [ign-1]: process has died [pid 131, exit code -11,
        cmd 'ign gazebo -s -r .../husarion_office.sdf --force-version 6']
```

A GPU device-removal event inside the WSL D3D12 driver, then a crash in the
renderer. **The container stayed up and every ROS node kept running** — the
only symptom was sim time frozen at t=550 s, exactly the "frozen clock with a
live container" stall the goal doc names. Both rovers' trajectories stopped at
the same sim timestamp, which is the tell: a wedged rover stops moving, a dead
simulator stops *time*.

This has now happened at the end of all three husarion runs. It is systematic,
not bad luck: two 640×480 RGBD cameras rendering through the WSL D3D12 path for
10+ minutes. Not a stack bug — an infrastructure limit.

**Three fixes landed as a result:**

1. **Clock-stall detection in the harness.** Two consecutive polls with an
   unchanged sim timestamp → declare the simulator dead, grep the container log
   for the D3D12/segfault signature into the run log, and go straight to map
   saving. A crashed run now costs about a minute instead of the full 25.
2. **The harness saves the merged map with `save_map_volatile.py`**, not
   `map_saver_cli`. The stock tool subscribes TRANSIENT_LOCAL only and can
   *never* receive `/shared_map` from a VOLATILE publisher.
3. **`scripts/run_snapshot.sh`** — runs a long script from an immutable copy so
   editing the original mid-run cannot corrupt execution. This is the third
   time that bit; it can no longer happen.

**Phase 1 gate: 2 of 4.** No-cheat proof holds and is now genuinely honest
(`zero_origin`); both rovers explored; alignment fails the 0.5 m / 10° bound;
merged map fails the eye test. Per the goal doc this is still a *usable* Phase 1
— "if the transform never converges but everything else works, record it as
such with the tag-detection counts per rover, and carry the map-matching
cross-check as the fallback" — with per-rover tag counts leo1 6, leo2 3,
3 common.

**Next, in order:** (a) iterate alignment **offline** on run 3's saved maps with
`fuse_maps_offline.py` — 3 common landmarks with ~0.5 m of SLAM-drift error in
each is a poorly conditioned Kabsch fit, and the tag/map disagreement (111° vs
65°) suggests the hybrid gate is accepting a bad map match; no re-simulation
needed. (b) Cut camera resolution to reduce the D3D12 crash rate before
spending another 25-minute slot. Phase 2 has not started.

### 2026-08-24 00:50 — tick 6b — offline diagnosis: **the blocker is leo2's SLAM, not the alignment**

No simulation. Ran the Kabsch fit myself over the two saved ArUco registries
from run 3 (`aruco_registry_leo{1,2}.json`), reprojecting leo2's landmarks into
the world through the known true spawn offset. This is the whole answer.

**leo1's landmarks are excellent — the detector is not the problem:**

| id | estimated (world) | truth | error | hits |
|---|---|---|---|---|
| 2 | (1.78, 0.00) | (1.79, 0.00) | **0.01 m** | 253 |
| 6 | (1.51, −5.40) | (1.47, −5.42) | 0.04 m | 65 |
| 5 | (13.35, −5.84) | (13.33, −5.90) | 0.07 m | 66 |
| 9 | (8.55, −7.40) | (8.52, −7.48) | 0.09 m | 8 |
| 8 | (6.22, −4.30) | (6.28, −4.41) | 0.13 m | 144 |
| 0 | (6.63, −11.88) | (6.51, −11.97) | 0.15 m | 41 |

Six landmarks, worst error **0.15 m**, best 1 cm, from a real ArUco detector on
a rover with corrupted wheel odometry. `marker_length = 0.20` and
`frame_is_optical = false` are settled beyond argument.

**leo2's are wrong by metres:** id 9 by 2.97 m, id 0 by 5.43 m, id 8 by
**8.80 m** — and id 8 had 186 hits at 3.05 m best range, so it is not a
detection problem, it is where leo2 *thinks it is*.

**`leo2_map.png` shows why: leo2's SLAM diverged.** The map is not a drifted
office, it is a shattered one — no continuous walls anywhere, just scattered
fragments and short segments at inconsistent angles, 1357 occupied cells
against leo1's ~3700 in a coherent map. Scan matching lost lock and never
recovered. Its landmark map is downstream of that, and so is every alignment
estimate built from it.

**The most important finding of the night is why nothing caught this.** The
Kabsch fit over the 3 common landmarks returns `(7.11, −10.04, 65.9°)` against
a truth of `(9.60, −9.95, 180°)` — 2.49 m and **114° wrong** — with per-tag
**residuals of 0.06, 0.09, 0.06 m**. Three points map onto three points almost
perfectly at completely the wrong rotation. `compute_tag_alignment_confidence`
sees three landmarks, sub-decimetre residuals and adequate spread, and reports
**0.82**.

> Residuals measure *self-consistency*, not correctness. With three landmarks
> and a distorted source map there is essentially always a rigid transform that
> fits them tightly and is wrong. A confidence built on residuals cannot see
> this failure mode, and it confidently reported success while being 114° out.

That generalises well beyond this run and belongs in the report.

**What would have caught it:** the tag and map-matching estimates *disagreed
violently* — 111° versus 65° yaw. `hybrid` currently fuses them and publishes
the blend. It should treat disagreement beyond a threshold as evidence that
neither is trustworthy and withhold the transform, which is exactly what
`alignment_tf_bridge` is built to act on. That is a concrete, testable change
and it needs no simulator.

**Why leo2's SLAM failed and leo1's did not — and the fix.** Identical
slam_toolbox parameters; the difference is that the realism harness feeds SLAM
**raw wheel odometry only**, with `yaw_scale = 0.12`. The *real* rover does not
do that — it runs an EKF fusing the IMU (`scripts/ekf_leo.yaml`, and the
hardware stack ships EKF), and the sim publishes `/leo{i}/imu/data` at 10 Hz
that nothing currently consumes. **So the sim is presently harsher than the
hardware it is supposed to model**, and it is losing roughly one rover in two
to heading divergence. Fusing the IMU is both the faithful fix and the one that
makes Phase 2 viable.

**Phase 1 conclusion.** The integration is sound and proven: build, tests, GPU,
per-robot SLAM topics, the real ArUco detector, the MarkerArray contract, the
persistent landmark map, the merger, and the pre-rendezvous degrade path all
work, with no ground-truth odometry, no identity offsets and no fixed alignment
mode. What is not proven is *alignment*, and the reason is a rover whose SLAM
diverges — not the tag pipeline. leo1's six landmarks at ≤0.15 m are the
evidence that the pipeline works when its input pose is sane.

**Next:** add IMU/EKF fusion so both rovers' SLAM survives, and add the
tag-vs-map disagreement veto. Both are cheap; the second needs no sim at all.

### 2026-08-24 00:50–01:18 — tick 7 — **PHASE 1 RUN 4: alignment 0.12 m / 1.1°, merged map CLEAN. Gate 3 of 4.**

Two fixes went in before this run, both derived from the run-3 diagnosis:

1. **IMU/EKF odometry, matching the real rover.** `sim_realism_odom.py` now runs
   with `publish_tf:=false`, `sim_realism_imu.py` degrades the gyro, and
   `robot_localization ekf_node` fuses wheel forward-velocity with gyro yaw
   *rate* and owns `odom -> base_link`. Configs generated per rover by
   `scripts/make_ekf_configs.py` from the existing reviewed `ekf_leo.yaml`
   (whose `/**` key matters: a bare `ekf_filter_node:` key silently matches
   nothing under a namespace and the filter would come up on stock defaults
   with no IMU at all). The IMU *orientation* is never fused — it is
   ground-truth-derived in Gazebo — only the rate.
2. **Tag-vs-grid disagreement veto** in `alignment_tf_bridge`: when the two
   independent estimates disagree by more than 2.0 m or 25°, publish nothing.
   Validated by replaying run 3's actual recorded estimates through it — 45.6°
   disagreement, veto fires, and both estimates were indeed outside the gate
   (2.58 m/111° and 3.58 m/65°). It would have prevented that run's
   confidently-wrong transform.

**Result — the alignment works.**

| | value | gate |
|---|---|---|
| tag-only estimate | `(9.7211, −9.9324, 178.89°)` | |
| **tag error vs truth `(9.60, −9.95, 180°)`** | **0.12 m / 1.11°** | 0.5 m / 10° ✅ |
| accepted (fused) estimate | `(9.7335, −9.7547, 179.17°)` | |
| accepted error | 0.24 m / 0.83° | ✅ |
| confidence at lock | tag 0.80, grid 0.67 | |
| common landmarks | 3 | `min_tags` 2 ✅ |

Convergence, from `alignment.csv`: no estimate until t≈335 s, then
**0.26 m / 0.3° at first estimate** (locked=0, confidence 0.41 — the bridge
correctly withheld), locking at t≈395 s once confidence passed 0.5, and stable
at 0.24 m for the remaining 5 minutes. **Recovered from tags alone, against a
real 13.8 m spawn offset, with no node ever told the answer.**

**Media review — PASSES.** `merged_map.png`: walls are **single-pixel and
continuous** — the long corridor wall at x≈8.8 is one line, not the three or
four of run 2. **No seam** where the two maps meet. **No grey speckle** in free
space. All geometry inside the world footprint (x −0.7…13.5, y −12…+1); nothing
outside the bounds. Both pillar clusters resolve as rings of discrete pillars,
the east rooms show their radiator fins along y≈−4.3, and the south rooms are
clean. It reads as one building mapped by one robot, which is the whole point.

`traj_overlay.png` is the night's headline image. leo1 (blue, 307 samples) from
the origin through the north-west room, south, and across the middle; leo2
(orange, 306 samples) from **(9.35, −9.8)** — its true spawn is (9.60, −9.95),
so the ~0.24 m alignment error is *visible on the picture* — north to y≈−3.5,
back down and west along y≈−9.5. Both tracks lie on free space and follow
corridors, which is the strongest possible confirmation the transform is right.
The two rovers worked **different regions**: leo1 north and west, leo2 east and
south. Coverage bears that out — leo1 73.8 m², leo2 74.6 m², merged 101.3 m²,
i.e. 47 m² of genuine overlap saved rather than duplicated.

(The overlay now maps leo2's full own-frame track through the recovered
transform. The shared-frame recorder cannot log a rover before the alignment TF
exists, so it had leo2 starting mid-run at 170 samples and understated what it
explored.)

**Gate: 3 of 4.**

| Gate item | Verdict |
|---|---|
| 1. Both explored, run ended on its own | **FAIL** — both explored thoroughly, but **the Gazebo server segfaulted again** (same D3D12 device-removal, 4 husarion runs out of 4) so the run ended on a dead simulator, not on completion |
| 2. `/shared_map` published, transform within 0.5 m / 10° from tags, with a confidence trace | **PASS** — 0.12 m / 1.11°, trace in `alignment.csv` and `alignment.png` |
| 3. Media review | **PASS** — clean walls, no seam, no phantoms, in bounds |
| 4. No gt odom, no identity offsets, no fixed alignment | **PASS**, and now genuinely honest: `zero_origin` + EKF, `cmdlines.txt` |

The one failure is **infrastructure, not the stack**: `ign gazebo` dies inside
`libd3d12core.so` after 9–13 minutes of two-camera rendering on the WSL D3D12
path. The new stall detector caught it in ~2 minutes and went straight to
salvage instead of burning the rest of the cap — that fix paid for itself on
its first run.

**Phase 1 is, in substance, done.** The two-rover stack was textually merged and
never run; it now runs end to end, recovers an unknown 13.8 m inter-map
transform from wall markers alone to 0.12 m, and produces a visually clean
merged map, with no ground-truth odometry, no identity offsets and no fixed
alignment mode anywhere.

**Before Phase 2:** cut the camera render load (drop `update_rate` 5 → 3 Hz in
the xacro) to reduce the D3D12 crash rate, since Phase 2 is three more maps.

---

## PHASE 2 — three-map matrix, one seed each

### 2026-08-24 01:21–01:46 — depot_world, coordinated — **run completed; merge FAILED**

**First run of the night to end on its own terms**: `all explorers finished`
(2/2), exit 0, **no Gazebo crash** (the 5→3 Hz camera change may be helping;
depot is also a lighter world), and `merged_map.pgm` **saved live** by the new
VOLATILE saver. Phase 1 gate item 1 — a run that terminates by itself — is
finally demonstrated.

Coverage (clipped to `-7.5 7.5 -7.5 7.5`): leo1 **136.6 m²**, leo2 **109.2 m²**,
merged 140.9 m².

**Alignment: tags right, grid catastrophically wrong.**

| estimate | value | error vs truth `(3.00, −9.00, 180°)` |
|---|---|---|
| **tag-only** | `(3.65, −8.57, 179.50°)` conf 0.76, 3 common tags | **0.78 m / 0.5°** |
| grid (accepted) | `(6.00, 0.10, −0.05°)` conf 0.67 | **9.58 m / 179.9°** |

The grid matcher found a confident **180°-flipped** match. That is exactly what
a rectilinear depot invites, and it is why tags are the primary sensor and grid
matching only the cross-check. The tag estimate's yaw is right to **0.5°**; its
0.78 m translation error is just outside the 0.5 m gate.

**The disagreement veto worked, live, and logged it plainly:**

```
withholding transform: tag vs grid disagree by 9.07 m / 178.1 deg
                       (limits 2.0 m / 25 deg)
```

It also *dropped* an earlier bad lock the moment a tag estimate appeared to
contradict the grid — the bridge had locked on the grid-only match at t≈245 s
(confidence 0.49, **zero** common tags) and released it at t≈735 s.

**But the merged map is still garbage, and that is the finding.**
`merged_map.png` shows a **doubled world**: depot is 14×14 m (x −7…7) and the
merged grid spans **x −7…16, y −11…2.5**, with the furniture pattern repeated
side by side. Two rooms where there is one.

**Cause — my veto protected the wrong consumer.** `alignment_tf_bridge` gates
the *TF tree*, so coordination was safe. `shared_map_merger` subscribes to
`/map_based_transform/leo2_to_leo1` **directly**, applies its own confidence
gate (0.67 > 0.5), and fused leo2 under the 180°-flipped grid transform — the
very transform the bridge was at that moment refusing to publish. Two consumers,
two independent decisions, one of them wrong.

**Fixes landed (rebuilt, 61/61 tests still green):**

1. **One vetted transform, two consumers.** The bridge now republishes what it
   has actually approved on `/vetted_transform/leo2_to_leo1` +
   `/vetted_alignment_confidence`, and `shared_align.launch.py` points the
   merger's `map_transform_topic`/`confidence_topic` at those. The grid the
   merger fuses and the frame the explorers coordinate in can no longer diverge.
2. **`require_tag_evidence` (default true).** No lock at all without a tag
   estimate, however confident the grid match looks. The depot lock happened
   with *zero* common tags, where `require_agreement` cannot help because there
   is no second opinion to disagree with.
3. **Tag-preferred disagreement policy.** When tag and grid disagree but the tag
   estimate is confident enough, publish the **tag estimate alone** rather than
   nothing. Withholding a 0.78 m tag estimate because a 9.58 m grid match
   disagrees with it costs all coordination for no safety gain.

**Verdict: depot coordinated = FAILED on the merge**, passed on exploration and
self-termination, and the tag alignment was good to 0.78 m / 0.5°. Re-run is
justified under the doc's list ("merged map shows doubled walls") *and* because
the cause is now fixed rather than guessed at.

### 2026-08-24 01:47–02:07 — depot_world, coordinated, **re-run** — merge **PASSES**, coordination does not

Re-run justified by the doc's list (doubled walls in the merged map) and by the
cause being fixed rather than guessed. `all explorers finished` (2/2), exit 0,
no Gazebo crash, merged map saved live.

**Alignment — inside the gate:**

| estimate | value | error vs `(3.00, −9.00, 180°)` |
|---|---|---|
| **tag-only (published)** | conf 0.80, **5 common landmarks**, locked | **0.23 m / 0.6°** ✅ |
| grid (rejected) | conf 0.73 | 7.79 m / 90.0° |

The grid matcher was wrong again — 90° this time rather than 180° — and the
veto again refused it, publishing the tag estimate instead. Two runs on the
same world, two different confident grid failures (180°, then 90°), tags right
both times. On a rectilinear depot, grid matching is not a usable primary.

**Media review — merged map PASSES.** `merged_map.png` is the depot, correctly:
outer shell x −7…7 and (in leo1's spawn-anchored frame) y −11.2…2.4, which maps
back to the true world y −6.7…6.9. Every structure is in the right place
against the ground-truth raster I made before the first run — the x=2 partition
and its y=2 return, the x=−2 partition, both y=0 wall stubs on the west side,
the short x=2 stub in the south, and all seven pillars/crates. **No phantom
rooms, no rotation, no seam, no speckle.** Compare the previous run's merged
map, which spanned x −7…16: a doubled world. The fix is visible, not inferred.

Walls do show **hairline doubling of roughly 0.2 m** on some segments — the two
maps offset by exactly the residual alignment error (0.23 m). It is visible at
full zoom and is honest to report; it is not the gross offset-parallel failure
of the earlier runs.

**Coordination did NOT pay off, and that is the result.** leo1 132.7 m²,
leo2 109.0 m², **merged 136.5 m²** — leo2 contributed only **3.8 m²** the other
rover had not already mapped. `traj_overlay.png` shows why: both tracks cover
the whole depot — west room, centre, east, north and south — heavily
overlapping. By the doc's table that is "trajectories tracing the same rooms",
i.e. coordination not actually engaged.

The mechanism is clear and is a genuine property of tag-based rendezvous, not a
bug: **coordination cannot begin until the rovers have seen common markers**,
because `_peer_xy` needs the alignment TF. Here that took until 5 common
landmarks accumulated, by which time both rovers had already been almost
everywhere — depot is only 14×14 m. **In a small world the rendezvous arrives
too late to be worth anything.** That is exactly the kind of null result the
goal doc says to report and move past, and it sharpens what the independent
baseline is for.

**Verdict: depot coordinated = PASS on mapping and alignment, null on
coordination.**

### 2026-08-24 02:09–02:38 — office_world, coordinated — **merge FAILED (37° rotation)**

Hit the 25-min cap with 1/2 explorers finished; all three maps saved.
Coverage (clipped `-12 12 -8 8`): leo1 **189.2 m²**, leo2 **214.4 m²**, merged
192.6 m². **6 common landmarks** — the most of any run — yet the published tag
estimate was `(13.79, −11.25, −143.0°)` against a truth of `(11.00, −10.00,
180°)`: **3.05 m / 37.0° wrong.**

**Media review — FAIL.** `merged_map.png` shows two building outlines rotated
~37° against each other and overlapping: wall pairs crossing at angles, the
south wall of one copy slicing through the middle of the other, and the merged
extent sprawling to y ≈ −23 for a world that ends at y = −8.

**Offline diagnosis (registries, no re-sim).** This time **leo1** is the rover
that drifted, and its error scales with distance from its spawn at (−7, 5):

| leo1 landmark | distance from spawn | error |
|---|---|---|
| id 3 `(−8.0, 7.88)` | 3.4 m | **0.03 m** |
| id 6 `(−11.88, −5.0)` | 11.2 m | 0.23 m |
| id 4 `(−3.88, 5.0)` | 3.1 m | 0.93 m (3 hits only) |
| id 1 `(−11.88, 0.0)` | 6.9 m | 1.93 m |
| id 2 `(11.88, 0.5)` | 19.4 m | **3.18 m** |
| id 5 `(11.88, 5.0)` | 18.9 m | **3.33 m** |

leo2's, by contrast, are mostly fine: id 2 **0.15 m**, id 8 0.10 m, id 4 0.21 m,
id 5 0.27 m, id 3 0.34 m, id 6 0.49 m; only id 1 is bad at 1.40 m. So the
drifting rover swaps between runs — leo2 on husarion run 3, leo1 here — and the
common factor is **traverse length**: ~3 m of SLAM error after ~19 m of driving,
even with the EKF. Note the EKF fixed catastrophic *divergence* (114°, shattered
map); it did not fix ordinary accumulating drift.

**And the aligner is throwing away usable information.** A plain Kabsch over the
node's *own* six common landmarks gives `(11.49, −8.48, −172.9°)` =
**1.60 m / 7.1°** — nearly twice as good as what the node published, and inside
the yaw gate. `merge_comparison.png` fuses the same two maps both ways: the
node's estimate is incoherent; the Kabsch fit is a recognisable office —
corridor band, three north rooms, two south rooms, partitions in the right
places — with wall doubling consistent with the residual 7°.

Per-tag residuals for that fit are 0.69–1.47 m, which straddles the aligner's
`max_tag_residual_mean` (0.75) and `max_tag_residual_max` (1.5). The gating is
almost certainly rejecting the full-set fit and falling back to a worse subset.
**The gates were tuned for landmarks that are accurate; they misbehave exactly
when drift makes the landmark set noisy but still collectively informative.**
That is a fix for the aligner, not for the night — recorded, not attempted.

**Verdict: office coordinated = FAILED.** Both rovers explored well and the
detector was fine; the merge is unusable because per-rover SLAM drift over a
24 m world exceeds what 6 tags can correct for, and the aligner's residual
gates make it worse rather than better.

### 2026-08-24 02:41–02:58 — depot_world, **independent baseline** — the honest comparison

Both explorers self-terminated, exit 0, no crash, all maps saved live.

| depot_world, n=1 each | leo1 | leo2 | **merged** | tag alignment | merge |
|---|---|---|---|---|---|
| coordinated | 132.7 m² | 109.0 m² | **136.5 m²** | 0.23 m / 0.6° | pass |
| independent | 132.2 m² | 108.7 m² | **134.5 m²** | 0.51 m / 2.8° | pass |

**Coordinated is 1.015x independent.** With n=1 that is a tie, and it sits
alongside the prior evidence in `reports/collab_final/` (0.95x on office, 0.90x
on depot) as another null result. Reported, not re-run.

Two things worth noting beyond the number:

- **The independent run's merged map is the cleanest of the night.**
  `merged_map.png` shows the depot with single-pixel walls throughout and no
  visible doubling at all — better than the coordinated run's, which carried
  ~0.2 m hairline doubling. Its alignment was *worse* on paper (0.51 m vs
  0.23 m) yet the picture is better, which is a good reminder that the merge
  quality depends on both rovers' map quality, not on the transform alone.
- **Alignment works the same in both conditions**, as it should: coordination
  changes where the rovers go, not how they recognise each other. Independent
  reached 4 common landmarks and 0.51 m / 2.8°; coordinated reached 5 and
  0.23 m / 0.6°.

---

## PHASE 3 — lab-ready: partly done

### 2026-08-24 ~03:00 — namespacing, DDS, marker card

**Done and verified to load** (`ros2 launch ... -s`), defaults unchanged so the
single-rover behaviour is byte-for-byte what the 2026-08-20 field runs used:

- `navigation_overlay.launch.py` — new `robot_ns` (default `rob_4`) and
  `cloud_input_topic`. The hardcoded `/rob_4/camera/depth/color/points` is now
  derived from `robot_ns`; with two rovers it pointed both cloud filters at the
  same machine's camera.
- `real_navigation.launch.py` — threads both through to the overlay.
- `real_exploration.launch.py` — namespaces `explore_node` and prefixes
  `robot_base_frame` to `{ns}/base_footprint`. Note `explore.yaml`'s
  `costmap_topic: map` is *relative*, so the namespace moves it to
  `/{ns}/map` for free; `robot_base_frame` is a **TF frame**, which namespaces
  do not touch, so left bare both rovers would look up the same frame and one
  would drive on the other's pose.

**Deliberately not done: `real_mapping.launch.py`.** It owns slam_toolbox —
which publishes to an absolute `/map` and needs the same load-bearing per-robot
remap `slam_multi.launch.py` carries in sim — plus the velocity guard and
collision monitor, which are the field-validated safety path. Changing that
without hardware to test against risks the one part of this tree that is known
to work. `LAB_SESSION.md` §4 lists exactly what it needs.

**`LAB_SESSION.md` written**: the marker card (DICT_4X4_50, ids 1–8, 200 mm
black square, ≥34 mm white border, 300 mm mounting height, `marker_length` 0.20,
`max_range` 4.5), the range/incidence table from the sim measurements, the DDS
plan (single domain 42, CycloneDDS peer list, explicit table of which topics
cross the network and which stay rover-local — the camera cloud is the one that
starves the firmware), the coordination decision (**physical spatial
partitioning, not runtime coordination**, argued from the null result), bring-up
order, six pre-flight checks, and a stall-symptom table.

**Not done: the lab-path rehearsal run** (office_world through the namespaced
real launches). `real_mapping` is the blocker.

**Final state: full `colcon build` 17 packages clean, 61/61 unit tests pass.**

### Deliverables

| item | state |
|---|---|
| `PROGRESS.md` ledger | done |
| `REPORT.md` | done |
| Phase 1 media set + convergence trace | done |
| Phase 2 three maps + comparison + coverage curves | done (+ independent baseline) |
| `LAB_SESSION.md` | done |
| Rehearsal run | **not done** |
| Every merged map as `.pgm`/`.yaml` **and** `.png` | done |
| Headline figure: merged map + both trajectories + markers | done (`traj_overlay.png`) |

### 2026-08-24 18:47–19:17 — depot_world, coordinated, **fully instrumented showcase run**

Run specifically to record the process, with the recorder extended to capture
goals, frontier candidates, tag sightings and alignment confidence alongside
the three occupancy grids, every 4 s. Also fixed: `publish_debug_image` was
never reaching the detector, so **no marker-annotated camera frames had been
captured all session** — the argument was missing from the harness. Now 16
annotated frames per rover plus 16 raw.

**Result: 0.31 m / 1.4°**, confidence 0.83, 4 common landmarks. Coverage leo1
132.7 m², leo2 109.7 m², merged 143.5 m². Merged map is the depot, correctly,
with ~0.3 m wall doubling matching the residual alignment error.

**It demonstrated the collinearity finding live, which is the most useful thing
it did.** For the first six minutes after both rovers had tags, the only two in
common were **1 `(0, 6.88)` and 6 `(0, −6.88)` — both on the x=0 wall.** Two
collinear landmarks leave the rotation undetermined, tag confidence sat at
**0.35**, and the bridge correctly refused to lock. At t≈406 s marker **4
`(6.88, 0)`** became common — off that line — and confidence jumped to **0.78**
and it locked immediately, converging to 0.31 m.

That is the placement rule from the lab card, observed rather than argued: the
shared markers must enclose area, not describe a line.

Three films built from the recording (`dashboard.html`): the merge (both
per-robot maps and the shared map side by side), goal selection (frontier
candidates and the chosen goal per rover), and the rendezvous (which tags each
rover has found and which are common). All scrubbable.

