# Demo visuals — what to run where (lab 2026-08-25)

**Name the rovers `leo1` and `leo2`.** Every launch file, explorer config,
aligner default and these RViz configs then work verbatim as sim-tested.
`rob_a`/`rob_b` would need edits in four places — don't.

## Screens

| where | command | shows |
|---|---|---|
| Laptop, screen-recorded (OBS) | `rviz2 -d config/rviz/demo_laptop.rviz` | both maps, shared map, frontiers, goal claims, ArUco landmarks, odom trails |
| Projector | `python3 scripts/live_merge_watch.py` | waiting-for-rendezvous → snaps to the merged map at lock |
| On each rover (or bag replay later) | `rviz2 -d config/rviz/demo_rover_local_leo1.rviz` (`_leo2` on leo2) | local costmap, scan, footprint, plan, odom |

## The laptop config, display by display

- **leo1 map** (gray) is drawn from the start. **leo2 map** (colored) is in
  `leo2/map` frame — RViz cannot place it until the alignment TF exists, so
  it **pops into view at the moment of lock**. That is the demo moment;
  don't "fix" it.
- **shared map** = the central merger's `/shared_map`. In distributed mode
  enable the `leo1 shared map` display instead (both are in the config).
  QoS matters and is pre-set: `/leoX/map` are Transient Local, all shared
  maps are **Volatile** — changing durability breaks reception silently.
- **goal claims** (`/exploration_claims`) is your goal-allocation visual:
  each rover marks the frontier it has claimed so the other avoids it.
- **aligner candidate (debug)** overlays where the matcher currently thinks
  leo2's map goes — enable it while waiting for lock if you want to narrate
  the abstention ("it has a guess, it is not confident enough yet").
- **Scans and costmaps stay OFF in the laptop config** (marked
  `WIFI LOAD - keep off`). Streaming them live over WiFi is the DDS load
  that has starved rover firmware (d241087). They are recorded on the
  rovers instead — see below — and filmed in replay.
- Fixed frame is `leo1/map`. If RViz shows "Frame does not exist" at start,
  the rover simply hasn't moved enough for SLAM to publish yet — normal.

## Recording

On each machine, before the run starts:

```bash
# on leo1:            on leo2:              on the laptop:
bash scripts/record_demo_bag.sh leo1
                      bash scripts/record_demo_bag.sh leo2
                                            bash scripts/record_demo_bag.sh laptop
```

Rover bags carry the heavy local topics (costmaps, scans, cmd chain);
the laptop bag carries only what already crosses the network. Plus OBS on
the laptop RViz + projector, and a phone on the rovers in the corridor.

## After the run — the film cut

1. Costmap/scan footage: `ros2 bag play <rover bag>` +
   `rviz2 -d config/rviz/demo_rover_local_leoX.rviz`, screen-record.
2. Merge/goal films and stills: the recorded maps feed the same pipeline as
   the sim runs (`scripts/render_multirobot_media.py`,
   `scripts/build_multirobot_dashboard.py`); `scripts/drive_replay/` turns
   bags into the dashboard directly.
3. Reference for what good output looks like:
   `reports/night_2026-08-25/dashboard.html`.
