# Debug tooling for real-rover exploration runs

Session tools developed on jetson-04 (2026-08-14). All run ON the Jetson with
`source /opt/ros/humble/setup.bash` and the robot's `ROS_DOMAIN_ID` exported.
Copy with `scp`, then strip CRLF (`sed -i 's/\r$//' <file>`) — see the CRLF
warning in REAL_ROVER_GUIDE.md.

## Recording a run — the two commands

```bash
python3 tools/debug_color_throttle.py            # terminal A, leave running
bash   tools/record_rover_bag.sh explore_run3    # terminal B, Ctrl+C to stop
```

`lean` (the default profile) is everything but depth, ~20 MB/min. Add `full`
as a second argument to include throttled depth, ~45 MB/min — only worth it
when the camera costmap layer has to be replayed. `SPLIT_MB` (default 1024)
and `COMPRESS=none` are the other knobs.

### Where the bytes go

Measured on `drive_2026-08-20`: **723 MB for 9.6 min = 75 MB/min**, one file,
nothing throttled.

| share | topic | per message | rate |
|---|---|---|---|
| 48% | `/bag/depth/compressed` | 190 kB | ~3 Hz |
| 44% | `/bag/color/compressed` | 157 kB | ~3.5 Hz |
| 3% | `/scan` | 4 kB | 10 Hz |
| 2% | the two `camera_info` topics | 0.4 kB | 30 Hz |
| 3% | odom, TF, IMU, cmd chain, wheel states | — | — |

92% is the two camera streams. The entire robot-state picture — the part that
makes the map, the path, the costmap film and the safety audit — costs about
**6 MB/min**, so an hour of it is 350 MB. The levers, in order of payoff:

1. **Drop depth** unless replaying the camera costmap layer. Halves the bag.
2. **Record the driver's jpeg, never the raw Image.** `/debug/color_5hz` is
   `sensor_msgs/Image`, rgb8: 921 kB/frame, 4.6 MB/s at 5 Hz — the raw
   republish was always a CPU fix, never a size fix. Its `/compressed`
   sibling is ~150 kB/frame for the same picture.
3. **Throttle `camera_info`.** At 30 Hz it costs more than `/cmd_vel`, `/tf`
   and the IMU combined, and replay needs one message per second at most.
4. **zstd the bag.** Nothing for jpeg/png payloads, roughly halves scans,
   costmaps, odometry and TF.
5. **Split the file** (`--max-bag-size`). A 4 GB single `.db3` is a bad thing
   to have to `scp` off a Jetson.

### Two topics that must never be recorded

* `/cmd_vel_raw` — the safety gate audits its subscribers and **closes** on an
  unexpected one (`unexpected raw command consumers: rosbag2_recorder`).
  Recording it stops the robot. A `ros2 topic echo` does the same.
* raw image topics — bagging 640x480 color+depth raw pushed load average to
  9.3 on 6 cores and opened 0.4–0.5 s scan arrival gaps that tripped the
  explorer's watchdogs.

`record_rover_bag.sh` lists topics explicitly rather than by regex for exactly
this reason, and skips any that are absent (namespaces have differed between
jetson-02 and jetson-04).

## debug_color_throttle.py

Publishes `/debug/color_5hz/compressed` (the driver's own jpeg, passed through
untouched) plus `/debug/color_5hz/camera_info`. `HZ` sets the rate cap
(default 5), `COLOR_TOPIC` the source. `RAW=1` additionally republishes the
raw `sensor_msgs/Image` on `/debug/color_5hz` — only for a tool that cannot
decode jpeg; do not bag it.

## record_rover_bag.sh

The recording command above. Profiles `lean` / `full`, message-level zstd when
`rosbag2_compression_zstd` is installed, 1 GB splits, absent topics skipped,
`/cmd_vel_raw` and raw images excluded by construction.

## firmware_stability_monitor.py

Watches firmware health for 300 s: battery-telemetry rate per 30 s bin,
battery voltage, and `enP8p1s0` traffic to/from the rover SBC. Baselines:
idle ~24 KB/s; full mapping stack ~85 KB/s; the documented firmware-starvation
incident was ~1400 KB/s. Battery telemetry must hold 10 Hz in every bin.

    nohup bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=4; \
      exec python3 firmware_stability_monitor.py' >/tmp/fw_monitor.log 2>&1 &

Note: firmware topics are best-effort QoS and `/rob_2/firmware/wheel_odom` is
`leo_msgs/WheelOdom`, NOT `nav_msgs/Odometry` — this script intentionally only
counts battery messages for liveness. `ros2 topic hz` is the reliable probe.

## render_labeled_debug_video.py

Renders an analysis video from an exploration rosbag (edit the BAG/OUT paths
at the top): camera frames + decision banner (DRIVING / CM SLOWDOWN /
OBSTACLE: CM HOLDING) derived from `/cmd_vel_request` vs `/cmd_vel`, top-down
fused-LIDAR + camera-scan panel with the collision footprint, explorer
mode/battery from `/rosout`, path length, gate closure reasons. ~2000 frames
render in ~4 min on the Jetson. Takes the colour frames from
`/debug/color_5hz/compressed`, falling back to the raw `/debug/color_5hz` for
bags recorded before 2026-08-27.

## What else the bag is not for

A bag is a bad container for a coverage curve or a final map, and recording
one is not the way to get them. On the rover these come from the stack itself:

* `ros2 service call /save_mapping_artifacts std_srvs/srv/Trigger '{}'` →
  `~/leo_maps/` (checkpoint any time while SLAM is alive)
* `nav2_map_server map_saver_cli` for the final map
* `/slam_toolbox/serialize_map` for a resumable pose graph
* `deploy/jetson02/cam_sampler.py` writes the driver's jpeg frames straight to
  disk at 2 Hz with no decode — ~350 MB for 30 min, ~0 CPU, and no bag needed
  if all you want is presentation stills.
