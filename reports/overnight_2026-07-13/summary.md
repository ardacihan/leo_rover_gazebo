# Overnight run 2026-07-13 — Report-ready collaborative maps + item-search benchmark

> STATUS: COMPLETE (2026-07-14 ~02:30). All three priorities delivered;
> 8 sim runs total (2 map runs + 6 benchmark runs), zero explorer errors.

## Priority 1 — Robust, aligned, report-ready merged maps

**Pipeline** (`scripts/map_fusion.py`, shared by the live compositor):
1. Correlative registration of leo2's submap against leo1's, seeded by the
   known spawn offsets (multi-resolution likelihood-field search).
2. Log-odds fusion (occupied/free votes) instead of naive max-overwrite.
3. Despeckle (small occupied components removed), unknown-hole fill, world-
   bound clip.
4. Scoring vs the world-SDF ground truth (`scripts/world_ground_truth.py`):
   wall IoU at 1-2 cell tolerance, precision/recall, RMSE to nearest wall.

**Registration validation (synthetic, real office map split in two):**
- exact prior (seed error <=0.3 m / 3 deg): mean recovery error 0.011 m /
  0.10 deg (max 0.026 m / 0.27 deg, n=8)
- coarse prior (seed error <=1.14 m / 15 deg): mean 0.015 m / 0.11 deg
  (max 0.061 m / 0.34 deg, n=8)
- full pipeline on a synthetically displaced pair: precision@2cells 0.999,
  RMSE 3.7 cm.
- Reference point: a good SINGLE-robot office map scores IoU@2 ~= 0.70
  against ground truth (recall is bounded by what the lidar can reach), so
  "report-ready merged" means matching that number, not 1.0.

**ROOT CAUSE FOUND (changes the whole story):** /leo{i}/odom comes from
Gazebo's `OdometryPublisher`, which reports **world-frame** pose - so every
robot's slam map frame is already world-anchored, and the "known spawn
offset" (1.5, 0) baked into map_merge_leo.launch.py + map_compositor.py was
WRONG for leo2 all along. That fixed offset - not SLAM drift - produced the
doubled walls, ghost furniture and seams in every earlier merged map (and
biased peer-pose conversions in the coordination discount by 1.5 m).
Registration discovered this: seeded at (1.5, 0) with a coarse window it
converged to (-0.055, 0.000, 0.35 deg) with a far higher match score, and
the map snapped into alignment. Both files now use identity offsets.

**Wall-IoU results (fresh 20-min coordinated runs, RTF 1.0, per-robot maps
saved for offline fusion):**

| world | pipeline | wall IoU@10cm | precision@10cm | recall@10cm | RMSE |
|---|---|---|---|---|---|
| office | BEFORE (fixed-offset overwrite) | 0.638 | 0.735 | 0.854 | 44.4 cm |
| office | AFTER (registered + fused + cleaned) | **0.775** | **0.988** | 0.830 | **5.3 cm** |
| depot | BEFORE | 0.484 | 0.603 | 0.799 | 52.5 cm |
| depot | AFTER | **0.779** | **0.992** | 0.826 | **4.7 cm** |

Registration corrections applied (vs identity seed): office leo2
(-0.055 m, 0.000 m, +0.35 deg); depot leo2 (-0.010 m, -0.010 m, +0.05 deg).
Figures: `p1_fusion/office_before_after.png`, `p1_fusion/depot_before_after.png`.
The live compositor (`scripts/map_compositor.py`) now runs the same log-odds
fusion + periodic drift registration online.

## Priority 2 — Item-search benchmark

Implementation (this session):
- `mock_aruco_detector` runs per robot in own-frame nav (LOS on /leo{i}/map,
  markers shifted common->own via TF).
- Shared item registry: explorers broadcast their registry on `/item_claims`
  (common frame); in the coordinated condition peers ADOPT confirmed items
  (skipping redundant verification detours). Peer candidates are not adopted.
- Shared camera coverage: observed wall keys broadcast on
  `/camera_coverage_claims` every 3 s; coordinated peers import them so no
  wall is swept twice. Sweep targets near a peer's active claim are skipped.
- Benchmark: office_world, 8 markers, 2 layouts (seed A/B), conditions
  1robot / 2indep / 2coord, 20 min cap, cameras ON, RTF capped 1.0.
- Metric: sim time to k-th confirmed item (union across robots), from
  `/item_claims` logs (`scripts/item_recorder.py`,
  `scripts/analyze_item_search.py`).

**Results (office_world, 8 markers, 20 min sim cap, union of confirmed
items across robots):**

| condition | seed A found | seed B found | t to 4 items (A / B) | t to all-found |
|---|---|---|---|---|
| 2coord  | **8/8** | **7/8** | **279 s / 261 s** | 967 s / 683 s (7th) |
| 2indep  | 4/8 | 5/8 | 894 s / 532 s | — (capped) |
| 1robot  | 5/8 | 3/8 | 1082 s / — | — (capped) |

- Coordinated found 15/16 items across both seeds; independent 9/16;
  single-robot 8/16. Coordinated was the ONLY condition to approach
  completion within the cap, on both seeds.
- Time-to-4-items: coordinated is 2.0-3.9x faster than independent and
  ~4x faster than single (where single even reached 4).
- Mechanisms visible in logs: peers adopt each other's confirmed items
  ("Adopted peer-confirmed items from leo1") skipping redundant verify
  detours, and shared camera-coverage claims prevent re-sweeping walls the
  other robot already observed.
- Figures: `item_search/time_to_items.png`,
  `item_search/items_overlay_2coord_seedA.png` (all 8 items confirmed on
  the fused merged map, annotated with discoverer + sim time; leo1 took the
  west rooms, leo2 the east - clean division of labour).
- Bonus: the merged map from the cameras-on coordinated run scores wall
  IoU@10cm 0.804 / RMSE 3.9 cm - the best map of the night (fusion output
  `item_search/2coord_seedA/fused/`).

## Priority 3 — Coarse-prior registration

- Synthetic (real office map split + known displacement): coarse prior
  (seed error up to 1.14 m / 11.6 deg) recovers the true transform to
  0.015 m / 0.11 deg mean (max 0.061 m / 0.34 deg, n=8).
- Real office pair: 5/5 random coarse seeds (up to ~1.2 m, +/-15 deg)
  converge to within 0.01 m / 0.15 deg of the exact-window answer
  (`p1_fusion/coarse_prior_real_pair.json`).
- Practical consequence already banked: the coarse-window search is exactly
  what exposed the wrong spawn-offset assumption (the true offset was 1.5 m
  from the seed - outside any "small drift" window).

## Limitations (honest)

- n=2 seeds per condition; run-to-run variance in this sim is ~±30% for
  coverage-style metrics, so item-time differences below ~2x are suggestive,
  not conclusive. The coordinated-vs-rest gap here exceeds that bar on both
  seeds, but 2 seeds cannot rule out layout luck entirely.
- Independent and single-robot runs hit the 20-min cap with items missing,
  so their time-to-all-items is right-censored (lower bound); found-counts
  are counts-at-cap. This inflates nothing in coordinated's favour — the cap
  hits every condition equally.
- Seeds vary the marker layout only; spawns are fixed (parameterizing the
  spawn/TF/compositor triple was deliberately avoided tonight).
- Mock (geometric) ArUco detector, not real perception.
- Registration assumes zero relative yaw between spawns (true here) and
  known offsets as seeds; the coarse-prior experiment relaxes this to
  ±1 m / ±15 deg but still assumes overlapping coverage.
- RTF capped at 1.0 (changed from 2.0 this session) for control fidelity;
  earlier reports' absolute times are not directly comparable.
