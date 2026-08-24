# Two rovers, one merged map — the 6-hour plan (2026-08-25)

**The essential deliverable is a merged map from two real rovers. The chosen
path is: each rover maps alone on its own ROS domain, and the merge happens
offline on the laptop with `scripts/align_registries_offline.py`.** Everything
live-and-coupled (shared DDS domain, live merger, coordinated exploration) is
bonus, attempted only if the essential is in the bag.

## Why this path, in three facts

1. **Every piece of it is already validated.** The per-rover stack
   (`real_mapping.launch.py`, teleop-first) ran on hardware 2026-08-20. The
   ArUco detector is the real one (7/8 markers, 0 false positives on the
   detector-only test). The offline merge was benchmarked tonight on all
   recorded two-rover sim runs: on every EKF-era run the plain Kabsch fit over
   the two tag registries recovered yaw to ≤3° and translation to 0.2–0.6 m,
   and the fused depot maps pass the eye test (see
   `reports/multirobot_2026-08-23/*/offline_merged*.png`).
2. **The live pipeline's failure mode is gone by construction.** On the
   2026-08-24 phase4 depot run the *live* aligner rejected a 0.6 m-good fit on
   its own residual gates and kept publishing a stale 19°-wrong transform —
   garbage merged map from good data. Offline, the human looks at the
   leave-one-out table and the picture, drops the one bad tag
   (`--exclude 5` turned that run from 0.63 m into **0.24 m / 1.6°**), and
   re-fuses in one second.
3. **m-explore's `multirobot_map_merge` was already tried and rejected**: its
   `known_init_poses` params silently fail under Humble (recorded 2026-07-06),
   and its feature-matching mode is exactly the kind of opaque magic that
   cannot be debugged in a 6-hour window. Two independent grid-matching
   attempts in our own stack produced confident 180°/90° flips on rectilinear
   rooms. Tags are the only alignment signal that has repeatedly worked.

Independent (uncoordinated) exploration costs nothing: the sim result is that
coordination is a wash in room-sized spaces (1.015x, n=1, consistent with all
earlier nulls). Start the rovers in different areas — physical partitioning IS
the coordination. And **teleop-first**: autonomous exploration stalls ~1 run
in 4; a stalled rover in a 6-hour window is a driver, not a debug target.

## What the audience actually sees (this is live, not post-hoc)

Both rovers drive **at the same time** — separate domains isolate them, they
do not serialize anything. On the projector, `scripts/live_merge_watch.py`
watches the laptop directory that a 15-second rsync pull loop keeps fresh
(each rover's registry JSON rewrites every 5 s while its detector runs; save
the map periodically or script `map_saver_cli` on a timer):

```bash
# one per rover, on the laptop:
while true; do rsync -q rovA:/data/aruco_registry_rovA.json \
    rovA:/data/rovA_map.{pgm,yaml} /lab/merge/; sleep 15; done &
python scripts/live_merge_watch.py /lab/merge --interval 10 --refine
# open /lab/merge/live.html fullscreen on the projector
```

`--refine` polishes each tag transform by grid correlation confined to a
±0.5 m / ±4° trust region around it — that is what makes the fused walls
single-pixel instead of hairline-doubled (validated: map overlap goes to
~99% on every good recorded run, and it rescued the fresh office run to
0.40 m / 0.3°). It is the safe version of "match the corridors": *global*
grid matching is banned here because on rectilinear rooms it confidently
produced 180°, 90° and 65° flips; locally seeded by tags, the true optimum
is the only one in reach. The display also overlays the **combined landmark
map** — tags colored by who found them (both / rover 1 only / rover 2
only), which is the "each robot benefits from the other's discoveries"
frame in one picture.

The demo has a built-in dramatic beat: while the rovers have fewer than two
common markers the screen shows two separate half-maps growing side by side
with "waiting for rendezvous"; **the moment both have seen the corridor
markers, the two fragments snap into one building**, with the recovered
transform printed on the frame. That moment — the system discovering the
inter-robot transform from observations, with no configured offset and no
shared frame — is the distributed-mapping claim, demonstrated live.

The display is downstream-only: nothing feeds back to the rovers, so a bad
frame can never break the run, and every frame is recomputed from scratch so
the picture self-heals as the registries improve. This is the same
architecture the DDS bandwidth plan wanted anyway — only maps and tag
sightings leave a rover; all raw sensing stays onboard.

## The rovers in the same space: collisions and ghosts

- **They will not collide any more than with any other obstacle.** Each rover
  navigates on its own lidar; the other rover appears in the scan and gets
  inflated in the local costmap like any dynamic obstacle, and the
  field-validated collision monitor + velocity guard are running. Under
  teleop, the driver is the collision avoidance.
- **The real nuisance is mutual map-ghosting, not collision**: a rover that
  lingers in the other's view gets baked into that SLAM map as a blob/smear.
  slam_toolbox clears it when the area is re-observed after the rover moves
  on; a blob survives only where the spot is never rescanned. Mitigation is
  the partition itself: different halves, and cross the shared corridor at
  different moments (trivial under teleop). If a blob survives, re-drive past
  that spot before the final map save.

## Timeline (6 h)

| when | what |
|---|---|
| 0:00–0:45 | Tape ≥3 markers in the shared corridor (card in `LAB_SESSION.md` §1: DICT_4X4_50, ids 1–8, 200 mm black square, ≥34 mm white border, on two roughly perpendicular walls — NOT in a line). Measure one with a ruler. Pace out both start poses and write them down. |
| 0:45–1:30 | Bring up rover A alone (own domain, e.g. `ROS_DOMAIN_ID=41`). Pre-flight §5 of `LAB_SESSION.md`, esp. the 1.5 m marker sanity check against a tape measure. |
| 1:30–2:30 | Drive rover A (teleop) through its half + the shared corridor past all common markers. Save map + registry. **Checkpoint: one good map on disk.** |
| 2:30–3:30 | Same for rover B (`ROS_DOMAIN_ID=42`, can overlap in time with A since domains are isolated — but only parallelize if A's map is already saved). |
| 1:30 on | Laptop: rsync pull loops + `live_merge_watch.py` on the projector — runs for the rest of the session. |
| 3:30–4:00 | Final quality pass with `align_registries_offline.py` → leave-one-out table, `--exclude` a bad tag, final merged PNG. **Checkpoint: merged map picture.** |
| 4:00–5:00 | Buffer for the one thing that will go wrong. If everything held: re-drive the weaker map, or attempt the live bonus. |
| 5:00–6:00 | Presentation media: merged PNG, per-rover maps, marker photos, `build_multirobot_dashboard.py` if artifacts allow. |

## Exact commands

Per rover (see `LAB_SESSION.md` §4 for the full bring-up; the deltas that
matter):

```bash
export ROS_DOMAIN_ID=41            # rover A; rover B uses 42 — full isolation
# ... real_bringup + real_mapping as on 2026-08-20 (NO namespacing needed) ...
ros2 launch leo_nav2_exploration aruco.launch.py profile:=real \
    marker_length:=0.20 max_range:=4.5 \
    registry_file:=/data/aruco_registry_rovA.json \
    samples_file:=/data/aruco_samples_rovA.csv
# marker_length DEFAULTS TO 0.15 in this launch — 0.20 is mandatory.
# registry_file DEFAULTS TO EMPTY — without it there is nothing to merge.
ros2 run nav2_map_server map_saver_cli -f rovA_map   # per-rover /map is fine
```

Copy `rovA_map.pgm/.yaml`, `rovB_map.*` and both registry JSONs to the laptop,
then:

```bash
python scripts/align_registries_offline.py <dir_with_all_six_files> \
    --truth <paced_dx> <paced_dy> <paced_yaw_deg>     # truth optional but free
# read the leave-one-out table; if one tag is poison:
python scripts/align_registries_offline.py <dir> --exclude <bad_id>
```

Output: transform, per-tag residuals, PASS/FAIL vs your paced truth, and
`offline_merged.png`.

## Fallbacks, in order

1. **A tag is bad** → `--exclude` it (the leave-one-out table names it).
2. **Fewer than 2 common tags** → drive both rovers past the corridor markers
   again (registries update every 5 s while the detector runs; a re-drive of
   just the corridor takes minutes).
3. **Tags unusable entirely** → the paced-out start poses ARE a transform:
   `python scripts/fuse_maps_offline.py rovA_map rovB_map merged --tf DX DY YAW_DEG`.
   This cannot fail and still yields the demo artifact.
4. **One rover dies** → one good map + the story + the sim results is still a
   presentation. `reports/multirobot_2026-08-23/dashboard.html` has the
   scrubbable sim films.

## Bonus tier (only after the merged PNG exists)

- Live shared map: put both rovers + laptop on domain 42 (CycloneDDS peer
  list, `LAB_SESSION.md` §2) and run `shared_align.launch.py` on the laptop.
  Known risk: `real_mapping.launch.py` is NOT namespaced — both rovers publish
  `/map`. That is exactly the clobbering bug from 2026-07-06. Do not attempt a
  live merge without either namespacing it (untested!) or remapping one
  rover's `/map` at launch.
- Coordinated exploration: sim-only claim; show `dashboard.html`.
