# Overnight Goal — Two-Rover Integrated Stack: run it, prove it on 3 maps, make it lab-ready

**Branch:** `feat/multi-robot-integration`
**Session type:** autonomous overnight, single night, three sequential phases.
**Ledger:** `reports/multirobot_<date>/PROGRESS.md` — created in Phase 0, appended
at every phase boundary and every 30-minute check. **The ledger is the source of
truth for where you are.** If you lose context, read it first and resume.

---

## Mission

Take the two-rover stacks that were textually merged onto this branch on
2026-08-23 but **never run together**, wire them into one stack, get one honest
end-to-end run, then prove it on three maps, then make it the thing that can be
carried into the lab.

Two rovers. Different rooms. **They know nothing about each other at startup** —
no shared frame, no known offsets, no ground-truth odometry. They discover each
other by both observing the same wall-mounted ArUco markers, and from that build
a merged map while exploring.

### Explicitly NOT the goal

- **Beating the uncoordinated baseline is not a gate.** The existing evidence
  (`reports/collab_final/figures/summary.md`) is a null result: coordinated is
  0.95× uncoordinated on office, 0.90× on depot, n=1. Measure it, report it
  honestly, move on. A night that produces a correct, fair, visually-clean
  merged map on three maps and loses to the baseline is a **successful night**.
- Seeds. **One seed per map.** Re-run a single map only if its result looks
  *broken* (see "Weird enough to re-run"), never to chase significance.
- Three rovers, item search, soak runs, rendezvous SLAM without markers.

---

## Ground rules (violating these wastes the night)

### 1. Look at the pictures. Every run. Before you believe any number.

A coverage number cannot see a doubled wall. **Every run must produce, and you
must actually open with the Read tool:**

- `merged_map.png` — the merged occupancy grid, rendered
- `leo1_map.png`, `leo2_map.png` — the per-robot inputs to the merge
- `traj_overlay.png` — both trajectories drawn on the merged map
- `coverage.png` — coverage vs sim-time, both robots + merged
- 3–4 camera frames per rover from the run (tag detections drawn if available)

**Score the run FAILED on visual evidence alone, regardless of metrics, if you
see any of:**

| What you see | What it means |
|---|---|
| Doubled / ghosted / offset-parallel walls | alignment wrong — the whole point of the night |
| Seam band where the two maps meet | merge blending or transform drift |
| Gray speckle fields in free space | fusion overwriting instead of accumulating |
| Map geometry outside the world bounds | SLAM drift; coverage number is inflated garbage |
| A trajectory that stops and never moves again | rover wedged — check for a doorway |
| A trajectory that oscillates in place | Nav2 recovery livelock |
| Trajectories tracing the *same* rooms | coordination not actually engaged |
| Walls at 90° to where they should be | `frame_is_optical` wrong on the detector |

Write what you saw in the ledger in words, not just a pass/fail. "Merged map
clean, single-pixel walls, leo2's south corridor lands 15 cm north of leo1's"
is worth more than any table.

### 2. Compute discipline

- Machine: RTX 4060 Ti 16 GB, i7-14700 (28 threads). Use them.
- **Hard cap 25 min wall-clock per sim run.** Pass the timeout; never let a run
  babysit itself into the morning.
- **Verify the GPU is engaged in the first 2 minutes of the first run**
  (`nvidia-smi` must show the gz/ign process). Cameras are ON this night —
  ArUco needs them — so RGBD rendering is the GPU load. Without
  `/usr/lib/wsl/lib` on `LD_LIBRARY_PATH` the headless server silently falls
  back to llvmpipe and everything takes 6× longer for no reason.
- **Runs are sequential.** Two camera-on two-rover sims in parallel will not
  hold RTF ≥ 0.9, and control fidelity couples to RTF.
- **Parallelize everything that is not a sim:** builds, map rendering, figure
  generation, analysis across runs, doc writing.
- Iterate map fusion/alignment **offline on saved maps** (seconds per
  iteration). Never re-sim to test a merge change.

### 3. Honesty

Every claim in the final report traces to a file in `reports/multirobot_<date>/`.
If a phase is not reached, say so and say why. A short true report beats a long
one. State n=1 wherever it is n=1.

---

## The 30-minute check protocol

Run under `/loop 30m` (command at the bottom). **Each tick, in this order:**

1. **Read the ledger** `reports/multirobot_<date>/PROGRESS.md`. What phase, what
   is supposed to be running, when did it start.
2. **Is anything actually running?** `docker ps` — is `leo_sim` alive? If the
   container is gone and no phase is marked complete, the run died: capture
   `docker logs --tail 200 leo_sim` into the ledger and restart or move on.
3. **Is sim-time advancing?** `/clock` must be moving. A frozen clock with a
   live container is the classic silent stall.
4. **Is the run making progress?** These must all have grown since last tick:
   - `traj.csv` row count (rovers are moving)
   - `coverage.log` last coverage value (map is growing)
   - `explorer.log` last line timestamp (explorers are deciding)
   If trajectories grow but coverage is flat for 2 consecutive ticks, the rovers
   are driving in already-mapped space — that is a livelock, not progress.
5. **Alignment health** (Phase 1 onward): is
   `/estimated_transform/leo2_to_leo1` publishing, and what does
   `/alignment_confidence` say? Log the current estimate vs ground truth. If no
   transform has ever appeared after 12 min of run time, no rover has seen two
   common tags — note it, let the run finish, and fix marker placement rather
   than re-running blind.
6. **Elapsed vs cap.** Past 25 min → kill it, mark it timed-out with whatever
   evidence exists, and go to the next item. **Never let one run eat the night.**
7. **Append a ledger line**: timestamp, phase, what is running, the four
   progress numbers, alignment state, and your one-line judgement.
8. If a phase finished since the last tick: **do the media review immediately**
   (Ground Rule 1) and record the verdict before starting the next run.

**Stop the loop** (`ScheduleWakeup stop`) when Phase 3 is complete or the
morning report is written.

---

## Phase 0 — Build and smoke (target: ≤ 45 min)

1. Check out `feat/multi-robot-integration`, `colcon build`, resolve whatever
   the merge broke. Run the existing unit tests — `multi_robot_shared_mapping/tests/`
   and `leo_rover_exploration/test/` are pure functions, they must pass before
   anything is simulated.
2. Create `reports/multirobot_<date>/PROGRESS.md` and write the plan into it.
3. **One 3-minute two-rover smoke run** on `depot_world`, cameras on:
   - `ros2 topic info /leo1/map -v` and `/leo2/map -v` → publisher count ≥ 1 each
     (0 means the per-robot `/map` remap in `slam_multi.launch.py` regressed —
     that remap is load-bearing)
   - both `compute_path_to_pose` action servers present
   - `nvidia-smi` shows the gz process
   - `/leo1/tag_detections` and `/leo2/tag_detections` exist and are `MarkerArray`

**Gate:** builds, tests green, smoke topics present, GPU confirmed. If the build
cannot be fixed in 45 minutes, that is the night's finding — write it up.

---

## Phase 1 — One integrated run, to completion, no cheating (target: ≤ 3 h)

This is the phase that matters. Everything after it is repetition.

### The wiring work

1. **Spawn poses.** `two_robots_gpu.launch.py:44` hardcodes
   `default_spawns = [(0,0), (1.5,0), ...]` — 1.5 m apart at the origin, and
   there is no launch argument to change it. Add `leo1_pose` / `leo2_pose`
   arguments in the `"x,y,z,R,P,Y"` form that `two_robots.launch.py` already
   uses, and thread them through `sim_gpu_wsl.sh`.
2. **Markers in the GPU launch.** `two_robots_gpu.launch.py` spawns **no ArUco
   markers at all**. Port the `MARKER_POSES` spawn block from
   `two_robots.launch.py`. Marker ground truth for the other worlds already
   exists as `leo_rover_exploration/config/mock_markers_{office,depot}_world.yaml`
   (z = 0.3, yaw = outward wall normal) — reuse those positions.
3. **The right detector.** `two_robots.launch.py:254` launches
   `leo_rover_semantic_vision/aruco_detection_node`, which publishes a
   `std_msgs/String` and a TF — **`tag_based_map_aligner` cannot consume it.**
   Replace with `leo_nav2_exploration/aruco_detector`, one per rover, with
   `detection_topic:=/leo{i}/tag_detections`. It already speaks `MarkerArray`
   and is the only hardware-validated detector in the tree.
4. **`frame_is_optical:=false`** on both detectors. Gazebo's `rgbd_camera`
   stamps images with the link frame, not an optical frame. Getting this wrong
   rotates every detection by 90° and errors nothing.
5. **`marker_length:=0.1333`.** The sim plates are `0.20 m` wide
   (`models/aruco_*/model.sdf`) with a one-cell quiet zone each side: 6 cells
   total, 4 payload → `0.20 × 4/6 = 0.1333`. The detector defaults to `0.15`,
   which puts every tag pose **12.5% too far along the view ray** and silently
   corrupts the alignment this whole night is about. Also widen `allowed_ids`
   to cover id 0 where a world uses ids 0–9.
6. **Turn the cheats off.**
   - `gt_odom_tf:=false` (`two_robots_gpu.launch.py:206` defaults to `true`) so
     SLAM stops receiving Gazebo's world-frame pose as a perfect odom prior.
     `scripts/sim_realism_odom.py` owns that transform instead.
   - Do **not** launch `map_merge_leo.launch.py`. Its identity
     `map→leo{i}/map` static TFs are only correct under ground-truth odometry.
   - `shared_mapping_demo.launch.py` defaults `alignment_mode=fixed`,
     `enable_apriltag_detection=false`, `enable_tag_alignment=false`
     (`:349–353`) — i.e. it hands the merger the true offset. Run
     `alignment_mode:=tag` (or `hybrid` with the map-matching cross-check) and
     `compare_to_ground_truth:=true` so the error is *measured*, not assumed.
7. **Pre-rendezvous mode.** `frontier_explorer._get_peer_offset` (line 339)
   looks up `map → {peer}/map` and, on success, **caches it forever**. Two
   problems: before the first mutual tag sighting there is no such transform at
   all, and after it, a converging alignment estimate is frozen at its first and
   worst value. Fix both — degrade cleanly to independent exploration while
   unaligned, and refresh the offset as confidence improves.
8. **`min_tags: 2`** in `tag_based_map_aligner`. Starting in different rooms,
   two rovers will not both see two common markers for several minutes. Either
   place markers so a shared corridor guarantees it, or lower the threshold and
   lean on `map_based_aligner` as the cross-check. Decide deliberately and
   record the decision.

### The run

`husarion_office` — it is the only world with per-room spawns and marker poses
already authored (`two_robots.launch.py`, leo2 at `2.36, -11.27`), so it is the
cheapest place to get the first honest run. Cameras on. 25-minute cap.

### Gate — all four, or Phase 1 is not done

1. Both rovers explored and the run terminated on its own (not on the cap).
2. `/shared_map` was published, and the recovered leo2→leo1 transform is within
   **0.5 m and 10°** of ground truth, from tags only — logged with the
   confidence trace over time, not a single final number.
3. **Media review passes** (Ground Rule 1). Clean walls, no seam, no phantoms.
4. No identity assumption, no ground-truth odom, no fixed alignment mode
   anywhere in the launch. Prove it by quoting the actual command line into the
   ledger.

If the transform never converges but everything else works, **that is still a
usable Phase 1** — record it as such with the tag-detection counts per rover,
and carry the map-matching cross-check as the fallback into Phase 2.

---

## Phase 2 — Three-map matrix, one seed each (target: ≤ 3 h)

| map | why | coverage bounds (from `auto_collab_run.sh`) |
|---|---|---|
| `depot_world` | simple, fast, sanity | `-7.5 7.5 -7.5 7.5` |
| `office_world` | medium, rooms + corridor | `-12 12 -8 8` |
| `husarion_office` | **the complex office** | `-4 27 -15 4` |

Per map, **one seed**, three conditions if time allows, in this priority order:
**coordinated** (always) → **independent** (for the honest comparison) →
**single-rover** (baseline). If time runs short, drop `single` first; a
coordinated run with no comparison still proves the stack.

Each map needs authored per-room spawn poses (the rovers must start in
*different rooms*, not different corners) and wall markers. Reuse the mock
marker ground truth for office/depot; author depot's spawns from its bounds.

**Per run, produce and review the full media set. Every time.** Record in the
ledger: final merged area, time-to-90%-of-best, alignment error vs ground truth,
mean rover separation, and **the visual verdict in words**.

### "Weird enough to re-run" — the only justification for a second seed

- Merged map shows doubled walls but alignment error reports < 0.5 m
  (the metric and the picture disagree — one of them is lying)
- A rover wedged in the first 3 minutes and never recovered
- Coverage went *backwards*
- Coordinated map area differs from independent by more than 2×

Anything else — including "coordinated lost" — gets reported, not re-run.

---

## Phase 3 — Lab-ready (target: ≤ 2 h; this is where a real deployment is won)

Make it the thing that can be carried into the lab and run on two physical
rovers. The real stack is **single-robot end to end** today.

1. **Namespace the real stack.** `config/real/explore.yaml` uses
   `base_footprint` and `costmap_topic: map`; `real_exploration.launch.py` has
   no namespace argument at all. Add a `robot_ns` argument threaded through
   `real_navigation.launch.py`, `real_mapping.launch.py`,
   `real_exploration.launch.py` and the real configs, so the whole chain comes
   up under `/rob_a` and `/rob_b` with prefixed TF frames.
2. **De-hardcode `/rob_4`.** `navigation_overlay.launch.py` pins the cloud
   filter input to `/rob_4/camera/depth/color/points`. Make it an argument.
3. **The DDS story, which does not exist yet.** Two rovers plus a laptop on one
   LAN, default `ROS_DOMAIN_ID=0` and plain multicast. There is a recorded
   finding on this branch (commit `d241087`) that **our own DDS traffic starves
   the rover's firmware**. Decide and document: domain IDs, a CycloneDDS peer
   config, which topics stay rover-local (scans, clouds, costmaps) and which
   cross the network (`/{ns}/map`, `/{ns}/tag_detections`, claims, the shared
   map). Bandwidth is the constraint — do not ship "put everything on domain 0".
4. **Marker reconciliation for print.** Sim plates are `0.20 m` → 
   `marker_length = 0.1333`. The lab tapes up *printed* markers of a different
   size. Fix the dictionary (`DICT_4X4_50`, which `make_aruco_models.py` and
   `aruco_detector` agree on) and the printed size, state the resulting
   `marker_length` and mounting height as a single number the lab reads off a
   card, and reconcile against whatever tool generates the printed sheets.
5. **`explore_lite` has no coordination hook** and it is what the real rover
   runs. Either give it one, or make the deployment story explicit: physical
   spatial partitioning (different start rooms) plus shared map merging, with
   coordination as a sim-only result. **Write down which one you chose.**

### Gate

Re-run **one** map from Phase 2 — `office_world` — through the *lab-ready*
launch path (namespaced `rob_a`/`rob_b`, real-stack configs, sim sensors), and
reach the Phase 1 gate again. That is the closest thing to a lab rehearsal this
machine can produce. Full media review.

Then write `LAB_SESSION.md`: exact bring-up order for two rovers, what to type
on each, marker size + height + ids on a card, the DDS setup, what to check
before trusting anything, and what to do when it stalls.

---

## Verified landmines — do not rediscover these

- `slam_toolbox` publishes to absolute `/map`; the per-robot remap in
  `slam_multi.launch.py` is load-bearing. Check publisher counts after any
  launch change.
- Never use `multirobot_map_merge`'s known-init-pose parameters — the
  leading-slash params silently fail on this Humble build.
- Coverage from the compositor over-counts under drift. Always clip to the
  world bounds in the table above.
- Docker runs from **PowerShell**, not bash, with `--entrypoint bash`.
- `pkill` inside `bash -c` matches its own shell and kills it. Stale C++ Nav2
  nodes from a previous run poison the next one — verify a clean process table
  between runs.
- `husarion_office` has wedged rovers before with cameras off. Cameras are on
  this night, but watch the trajectories for a rover parked in a doorway.
- Files that carry the **only hardware evidence there is** and must not be
  casually edited: `config/real/*`, `config/real_baseline_2026-08-20/*`,
  `cloud_filter.py`, `scan_normalizer.py`, `rover_ws/jetson4/`. Phase 3 adds
  namespacing *around* them; it does not retune them.

---

## Deliverables — `reports/multirobot_<date>/`

- [ ] `PROGRESS.md` — the running ledger, every 30-min check appended
- [ ] `REPORT.md` — what was built, what ran, what the pictures showed, honest
      limitations (n=1, sim-only, mock vs real perception), and what is left
- [ ] Phase 1: the integration run's full media set + alignment convergence trace
- [ ] Phase 2: 3 maps × media set each + one comparison table + coverage curves
- [ ] Phase 3: `LAB_SESSION.md` + the lab-path rehearsal run's media set
- [ ] Every merged map saved as `.pgm`/`.yaml` **and** rendered `.png`
- [ ] One figure that shows a merged map with both trajectories and the marker
      positions on it — that single image is the night's headline
