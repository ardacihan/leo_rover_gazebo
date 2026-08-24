# Two-rover integrated stack — what ran, what it showed, what is left

**Branch:** `feat/multi-robot-integration` · **Session:** 2026-08-23 22:45 →
2026-08-24 · **Ledger:** `PROGRESS.md` (every run reviewed in words)

Every claim below traces to a file in this directory. Where n=1, it says n=1.

---

## The one-line answer

The two-rover stacks were textually merged onto this branch and **had never been
run together**. They now run end to end: two rovers start in different rooms
knowing nothing about each other, discover each other by both seeing the same
wall-mounted ArUco markers, and build a merged map — **with no ground-truth
odometry, no identity map offsets, and no fixed alignment mode anywhere.** On
`husarion_office` the recovered transform was **0.12 m and 1.1°** against a real
13.8 m spawn offset, and the merged map is visually clean.

It does **not** work reliably yet. It worked on husarion and depot and failed on
office, and the reason in every failure is the same: **per-rover SLAM drift, not
the tag pipeline.**

---

## Phase 0 — build and smoke: PASS

- `colcon build`: **17 packages, 45 s, zero errors.** The textual merge did not
  break the build. `multi_robot_shared_mapping`, `multirobot_map_merge`,
  `leo_rover_real_bringup` and `leo_rover_semantic_vision` had never been built.
- Unit tests: **61 passed**, and still 61 after every change this session.
- Smoke on `depot_world` (`smoke_depot/`): all 10 assertions pass — per-robot
  `/leo{i}/map` publishers, `/leo{i}/tag_detections` typed `MarkerArray`, both
  rovers confirming real markers, two `compute_path_to_pose` servers, all four
  alignment nodes up, ground-truth TF diverted off `/tf`.
- **GPU confirmed**: `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)`, not
  llvmpipe. (`nvidia-smi --query-compute-apps` on the Windows host cannot see
  WSL/container processes — the Ogre log is the check that decides.)

---

## Phase 1 — one honest integrated run: 3 of 4 gate items

Four runs on `husarion_office`. Run 4 is the result; runs 1–3 each failed for a
different reason, and each failure produced a fix.

| Gate item | Verdict |
|---|---|
| 1. Both rovers explored, run terminated on its own | **FAIL** — both explored thoroughly, but Gazebo segfaulted rather than the run completing |
| 2. `/shared_map` published; transform within 0.5 m / 10° from tags, with a confidence trace | **PASS** — **0.12 m / 1.11°** |
| 3. Media review | **PASS** |
| 4. No gt odom, no identity offsets, no fixed alignment mode | **PASS** — see `cmdlines.txt` |

**Convergence** (run 4, truth `(9.60, −9.95, 180°)`): no estimate until
t≈335 s → **0.26 m / 0.3°** at first estimate, with the bridge correctly
withholding at confidence 0.41 → locked at t≈395 s once confidence passed 0.5 →
stable at 0.24 m for the remaining five minutes. Tag-only estimate **0.12 m /
1.11°**, confidence 0.80, 3 common landmarks.

**Media** (`phase1_husarion_coordinated_run4/`): `merged_map.png` has
single-pixel continuous walls — the long corridor wall at x≈8.8 is one line, not
the three or four of earlier runs — **no seam, no speckle in free space, nothing
outside the world footprint**. `traj_overlay.png` is the headline figure: leo1
from the origin through the north-west room and across the middle; leo2 from
**(9.35, −9.8)** where truth is (9.60, −9.95), so the 0.24 m alignment error is
visible on the picture; both tracks lie on free space and follow corridors. The
rovers worked different regions — leo1 73.8 m², leo2 74.6 m², merged 101.3 m²,
about 47 m² of overlap avoided.

---

## Phase 2 — three maps, one seed each

| map | condition | explored | merged | tag alignment vs truth | merge verdict |
|---|---|---|---|---|---|
| `husarion_office` | coordinated | leo1 73.8, leo2 74.6 m² | 101.3 m² | **0.12 m / 1.1°** | **PASS** |
| `depot_world` | coordinated (re-run) | leo1 132.7, leo2 109.0 m² | 136.5 m² | **0.23 m / 0.6°** | **PASS** |
| `office_world` | coordinated | leo1 189.2, leo2 214.4 m² | 192.6 m² | 3.05 m / 37.0° | **FAIL** |
| `depot_world` | independent baseline | leo1 132.2, leo2 108.7 m² | 134.5 m² | 0.51 m / 2.8° | **PASS** |

n=1 per cell. Coverage is clipped to the world bounds everywhere; unclipped
numbers flatter drifted maps and are not comparable across conditions.

**depot passes and is worth reading closely.** Both explorers self-terminated,
no crash, merged map saved live from the running merger. The merged grid is the
depot, correctly — every partition, wall stub and pillar in the right place
against the ground-truth raster, no phantom rooms, no rotation. Walls show
**hairline doubling of about 0.2 m**, exactly the residual alignment error,
visible only at full zoom.

**office fails on rotation.** Six common landmarks — the most of any run — and
still 37° out, because leo1's landmark positions degrade with traverse length:
**0.03 m** at 3 m from its spawn, **3.33 m** at 19 m. The drifting rover is not
consistent between runs (leo2 on husarion run 3, leo1 on office); the common
factor is distance driven.

**Coordinated vs independent on depot: 136.5 m² against 134.5 m², i.e.
1.015x — a tie at n=1.** That sits alongside the prior evidence in
`reports/collab_final/` (0.95x on office, 0.90x on depot) as another null
result. The independent run's merged map is in fact the *cleanest* of the
night — single-pixel walls throughout, no visible doubling — despite a worse
transform (0.51 m vs 0.23 m), which is a useful reminder that merge quality
depends on both rovers' map quality and not on the transform alone.

**Coordination did not pay off, and that is a result, not a failure.** On depot,
merged 136.5 m² against leo1's own 132.7 m² — leo2 contributed **3.8 m²** the
other rover had not already mapped, and both trajectories cover the whole world.
The mechanism is inherent to tag-based rendezvous: coordination cannot start
until the rovers have seen common markers, because peer poses are a TF lookup.
On a 14×14 m world they have both been almost everywhere by then. **In a small
world the rendezvous arrives too late to be worth anything.**

---

## What was actually wrong, and how it was found

Five defects, each found by measurement rather than inspection, in order of how
much they mattered.

**1. SLAM was fed raw wheel odometry, which is harsher than the real rover.**
`sim_realism_odom.py` models a 12% skid-steer yaw-scale error, and nothing
consumed the IMU the sim already publishes. One rover's heading diverged by
**~114°** and its map shattered into fragments; every landmark it owned was
metres wrong, and alignment went with it. The physical rover does not run
open-loop wheel yaw — it runs an EKF fusing gyro yaw *rate*. Wiring
`robot_localization` in (configs generated per rover from the existing reviewed
`ekf_leo.yaml`) took alignment from **2.58 m / 111°** to **0.12 m / 1.1°** on
the same world. The IMU *orientation* is never fused: in Gazebo it is
ground-truth-derived, and fusing it would hand the filter the answer.

**2. Both SLAM maps were secretly in the world frame.** `sim_realism_odom.py`
seeded its integration on the true world pose, so `leo2/map → leo1/map` was
identity *by construction* and the two-rover problem was fake — an early run
scored 0.12 m against a target that was free. Added `zero_origin`, so each
rover's odometry starts at (0, 0, 0) as real wheel odometry does and the true
transform is the actual spawn offset. Same world-anchored-odom trap as
2026-07-13, in a new place.

**3. `marker_length` is 0.20 m, not the goal doc's 0.1333.** Measured: the
`aruco_N` textures carry **no quiet zone** — the black square fills 100% of every
one — so the square *is* the plate. Confirmed in sim against a known marker:
**1.5 cm error at 1.5 m range**. The doc's figure would have put every tag pose
33% short along the view ray and the detector's 0.15 default 25% short, with
nothing erroring.

**4. Alignment confidence is not a correctness signal.** Three common landmarks
fitted each other to **0.06–0.09 m residuals**, reported **0.82 confidence**, and
were **114° wrong** — the source map was distorted, and with three points there
is essentially always a rigid transform that fits tightly and is wrong.
Residuals measure self-consistency. The only available signal was that the tag
and grid estimates disagreed by 46°. That is now a veto, and it fired live on
depot and dropped a bad lock. Also added: **no lock without tag evidence at all**
(a grid-only match locked on depot with *zero* common tags at 179.9° error), and
**prefer the tag estimate when the two disagree**, since tags are the primary
sensor and grid matching only the cross-check.

**5. Coordination was wired to frames that exist only under the forbidden
cheat.** `frontier_explorer._peer_xy` and `._get_common_offset` are TF lookups
that resolved only because `map_merge_leo.launch.py` published *identity*
`map → leo{i}/map` statics — correct solely under ground-truth odometry. Without
a replacement, "coordinated" silently degrades to independent and the night
measures nothing. New `alignment_tf_bridge` publishes the recovered
`leo1/map → leo2/map` once trusted, refreshing as it improves, and publishes
**nothing** before that, which is the correct pre-rendezvous behaviour.
`_get_peer_offset`'s permanent cache became a TTL so a converging estimate is
not frozen at its first and worst value.

---

## Honest limitations

- **n=1 everywhere.** One seed per map, as instructed. No significance claimed.
- **Sim only.** Nothing here ran on a physical rover. The detector, the EKF
  topology and `marker_length` are the parts most likely to transfer; the SLAM
  drift numbers are Gazebo's, not a real floor's.
- **Gate item 1 is not met on husarion.** `ign gazebo` segfaults inside
  `/usr/lib/wsl/lib/libd3d12core.so` (`D3D12: Removing Device`) after 9–13
  minutes of two-camera rendering — **4 of 4 husarion runs**. The container
  survives and every ROS node keeps running, so the only symptom is a frozen
  clock. Infrastructure, not the stack; the harness now detects it in about a
  minute instead of burning the full cap, and the camera rate was cut 5→3 Hz.
- **husarion run 4's merged map was fused offline**, because the live
  `/shared_map` died with the simulator. It used the transform the run actually
  recovered. depot's merged map *was* saved live from the running merger.
- **The tag aligner discards usable landmarks.** On office a plain Kabsch over
  the node's own six common landmarks gives 1.60 m / 7.1°, against the 3.05 m /
  37° it published — see `phase2_office_coordinated/merge_comparison.png`. Its
  residual gates (`max_tag_residual_mean` 0.75) are tuned for accurate landmarks
  and misbehave when drift makes the set noisy but still collectively
  informative. Diagnosed, not fixed.
- **Phase 3 is partly done.** `navigation_overlay`, `real_navigation` and
  `real_exploration` now take `robot_ns` (and the hardcoded `/rob_4` depth
  cloud is derived from it), defaults unchanged so single-rover behaviour is
  byte-for-byte what the field runs used; all three verified to load, **none
  run on hardware**. `real_mapping.launch.py` was deliberately left alone — it
  owns slam_toolbox's load-bearing absolute-`/map` remap and the validated
  velocity-guard path, and changing it untested risks the one part of this tree
  known to work. The lab-path rehearsal run was not done. `LAB_SESSION.md` has
  the marker card, the DDS plan, the coordination decision and the exact
  remaining work.
- Two sim-only liberties: marker plates are textured on **both** faces (a
  printed marker is one-sided), and `min_tags` stayed at 2 rather than being
  lowered — markers were instead placed so a shared corridor makes two sightings
  likely.

---

## Where the effort should go next

1. **Per-rover SLAM drift is the binding constraint**, not alignment. ~3 m after
   19 m of driving is what breaks office. Loop-closure tuning, or a second
   correction source, buys more than any change to the tag pipeline. Note the
   EKF fixed catastrophic *divergence*; it did not fix ordinary accumulating
   drift.
2. **Fix the aligner's residual gating** so a noisy-but-informative landmark set
   is used rather than discarded. The office data is saved and the iteration is
   offline and instant (`scripts/fuse_maps_offline.py`).
3. **Coordination must start earlier than rendezvous** to be worth anything in a
   small world. Spatial partitioning from known start rooms is the deployable
   version and needs no shared frame at all.

---

## Reproducing this

Everything below runs from the repo root on a machine with Docker and the
`leo_rover_humble:bundle` image. Nothing needs a rover.

```bash
# 1. build (throwaway container, ~45 s)
docker run --rm -v "$PWD:/ros2_ws" -e ROS2_WS=/ros2_ws --entrypoint bash   leo_rover_humble:bundle -lc   "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# 2. sanity check before spending 25 minutes on a run
bash scripts/smoke_multirobot.sh depot_world reports/smoke 12

# 3. a full two-rover run. Always launch through run_snapshot.sh -- it executes
#    an immutable copy, so editing the harness mid-run cannot corrupt it.
bash scripts/run_snapshot.sh scripts/auto_multirobot_run.sh      coordinated depot_world reports/my_run 25
#    modes: coordinated | independent | single
#    worlds: depot_world | office_world | husarion_office

# 4. figures, marker map, and the dashboard
python scripts/render_multirobot_media.py reports/my_run --world depot_world
python scripts/render_marker_map.py reports/my_run --world depot_world        --out reports/my_run/marker_map.png --spawn1 0 4.5 0
python scripts/build_multirobot_dashboard.py reports --timelapse-dir my_run        -o reports/my_run/dashboard.html
```

Two things worth knowing before the first run:

- **Watch `/clock`, not just `docker ps`.** Gazebo segfaults inside the WSL
  D3D12 driver on long two-camera runs and the container stays up with every
  node running — the only symptom is that sim time stops. The harness detects
  it after two polls and salvages the maps.
- **Iterate merges offline.** `scripts/fuse_maps_offline.py` fuses two saved
  maps under any transform in about a second. Re-simulating to test an
  alignment change costs 25 minutes and buys nothing.

The raw per-frame camera capture and the time-lapse snapshots are gitignored
(bulky and regenerated by any run); the figures, maps, traces and the dashboard
are committed.

## Artifacts

| path | what |
|---|---|
| `PROGRESS.md` | running ledger, every run reviewed in words |
| `phase1_husarion_coordinated_run4/` | the Phase 1 result + full media set |
| `phase2_depot_coordinated_run2/` | depot coordinated, merge passes |
| `phase2_office_coordinated/` | office, merge fails + `merge_comparison.png` |
| `phase2_depot_independent/` | uncoordinated baseline |
| `smoke_depot/`, `smoke_husarion/` | Phase 0 smokes |
| `worlds/*.png` | rasterized worlds with spawns and markers checked by eye |
| `LAB_SESSION.md` | what the lab needs to know |

Per run: `merged_map.png`, `leo1_map.png`, `leo2_map.png`, `traj_overlay.png`,
`coverage.png`, `alignment.png`, `frames_leo{1,2}/`, plus `.pgm`/`.yaml` for
every map and `cmdlines.txt` recording exactly what was launched.
