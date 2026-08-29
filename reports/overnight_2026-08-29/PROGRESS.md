# Overnight validation — 2026-08-29

Branch: `feat/multi-robot-integration`

Deadline: 09:00 Europe/Berlin. Every simulation run is capped at 20 minutes.

## Acceptance gates

- Two robots start in authored different-room poses without a shared frame or
  fixed/ground-truth alignment.
- RGB camera, RGB-D point cloud, and GPU LiDAR are live for both robots.
- Gazebo renders on the WSL-visible NVIDIA GPU, not llvmpipe.
- Each robot explores independently before alignment; common ArUco landmarks
  and/or honest map evidence establish the transform; `/shared_map` then
  becomes available to both explorers.
- Coordinated exploration covers at least as much of each complete world as
  the two-robot independent baseline, while avoiding repeated work where the
  evidence supports that claim.
- Fresh local evidence covers `husarion_office` and at least one additional
  complete world.
- Final maps, trajectories, coverage/alignment plots, camera evidence, and a
  live merge/timelapse are presentation-ready and visually reviewed.

## Ledger

### 2026-08-28 22:37–22:55 — orientation and build

- Branch confirmed: `feat/multi-robot-integration`, tracking the corresponding
  origin branch. Existing dirty files and recorded rover data are user-owned
  and are being preserved.
- WSL2 kernel `6.18.33.2-microsoft-standard-WSL2` sees `NVIDIA GeForce RTX
  4060 Ti`, driver 610.62, 16380 MiB.
- Current packages rebuilt in the ROS Humble container: `multi_robot_shared_mapping`,
  `leo_rover_gazebo`, `leo_rover_exploration`, and `leo_nav2_exploration` all
  built successfully (28.5 s).
- Core shared-mapping tests: 49/49 passed. Collaborative explorer tests: 18/18
  passed. ArUco pose/calibration and most navigation tests passed; the same 13
  previously documented `leo_nav2_exploration` config/operator-bundle contract
  failures remain (79/92 passed). These failures are being kept distinct from
  the fresh simulation gates rather than hidden by a narrow aggregate.

### 2026-08-28 22:46-23:10 - camera-on depot diagnostics

- A six-minute camera-on smoke run kept both RGB streams, both organized RGB-D
  point clouds, and both 360-sample laser scans live. The bounded message probe
  passed all required topics; the Gazebo Ogre log reports the D3D12 renderer on
  the RTX 4060 Ti. The run correctly abstained from merging after only one
  weak common-tag observation.
- Fixed two presentation defects found from the fresh artifacts: OpenCV could
  not draw on the negative-stride vertically flipped map, and authored marker
  coordinates were plotted in world coordinates instead of the leo1 map
  frame. A final accepted transform is now also applied to the full leo2
  trajectory in overlays. The focused media suite passes 3/3.
- The 16-minute depot run is diagnostic, not a comparable result: its original
  alignment stack accepted a wrong grid-only state before rendezvous. Once two
  common tags were persistent, the tag solver recovered `(3.071, -8.890,
  179.7 deg)`, only 0.13 m / 0.3 deg from truth, but an inconsistent hidden
  0.35 m residual limit rejected its 0.67 m SLAM-distorted landmark residual.
- The tag seed gate is now consistent with the documented downstream limits
  (0.75 m mean, 1.5 m max). Hybrid fusion state is also prevented from anchoring
  to a pre-rendezvous grid candidate; map/tag disagreement is enforced for any
  available tag seed. Focused alignment, policy, and media tests pass 21/21.
- A live-only alignment-stack restart verified that mature depot occupancy maps
  independently recover about `(3.25, -8.90, 179.1 deg)`, 0.27 m / 0.95 deg
  from truth. The intervention is retained as diagnostic evidence only; clean
  end-to-end runs follow.

### 2026-08-28 23:15-23:56 - clean depot comparison

- Clean camera-on coordinated run: the pre-rendezvous grid candidates remained
  preview-only; two common persistent tags appeared at about 365 s simulation
  time; the bridge locked at about 370 s. The accepted transform remained near
  0.23-0.30 m and 0.7-1.1 deg error for the rest of the run.
- Immediately after lock, leo2 reported 4,161 locally unknown cells already
  covered by `/shared_map`, proving that the explorer consumed peer-covered
  space rather than merely drawing a merged map for presentation.
- Fresh depot head-to-head: coordinated 134.2 m2 final coverage, 122.7 m2
  duplicated area, 33 goals; independent 133.5 m2, 174.9 m2 duplicated area,
  42 goals. At the independent run's matched final timestamp the coordinated
  curve is still ahead, 134.1 vs 133.5 m2. Collaboration therefore preserves
  map result while reducing repeated mapping by 52.2 m2 (29.8%) and issuing
  nine fewer frontier goals.
- Both conditions passed full-stack RGB/RGB-D/LiDAR probes. Live NVIDIA-SMI
  during the coordinated workload showed 857 MiB allocated and 11% GPU use;
  Ogre independently named the RTX 4060 Ti D3D12 renderer.

### 2026-08-29 08:21-08:52 - fresh camera-on office comparison

- Coordinated local WSL run passed both-rover RGB, camera-info, organized
  RGB-D cloud and LiDAR message probes. Two common persistent tags appeared
  after independent exploration; occupancy validation locked at about 205 s
  simulation time. The final accepted transform was 0.18 m / 0.6 deg from
  authored truth, with three common landmarks and 0.86 tag confidence.
- Immediately after lock, leo2 suppressed 12,308 locally unknown cells already
  covered by the peer-backed shared map. The pre-lock and aligned phases were
  visually verified from the rendered live-merge video.
- Fresh matched office result: coordinated 103.7 m2 versus independent 88.1
  m2 (+15.6 m2, +17.7%). The independent rovers never rendezvoused, so their
  score is the offline union of both recorded local maps under authored truth;
  that evaluation transform was never published at runtime.
- The fresh independent paths were nearly disjoint (0.6 m2 overlap), so no
  duplicate-reduction claim is made from this short pair. The completed office
  pair remains 181.2 vs 179.8 m2 with 16.8% less duplicated area; fresh depot
  remains 29.8% less duplicated area.
- Both fresh office executions completed maps and media in 14m09s / 14m08s.
  The harness now enforces the requested 20-minute limit end-to-end rather
  than treating its argument as an exploration-only wall cap.
- Final shared-mapping suite: 58/58 passed, including 23 focused alignment,
  policy, presentation and no-rendezvous scoring regressions. Bash syntax,
  Python style/compile checks and `git diff --check` passed.
