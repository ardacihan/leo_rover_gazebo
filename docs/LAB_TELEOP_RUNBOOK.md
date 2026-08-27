# Lab card — drive by hand, record a bag

Two ways to drive. **The browser is the simple one** and needs nothing
installed; the keyboard one exists because it goes through the safety chain.
Recording is identical either way.

Everything except the browser runs **on the rover** over SSH, each block in its
own terminal, with `source /opt/ros/humble/setup.bash` and the right
`ROS_DOMAIN_ID` exported (rover 4 → 4, rover 2 → 2). Scripts copied from
Windows need `sed -i 's/\r$//' <file>` first. Rover 4 topic names are used
below; on rover 2 the firmware sits at the root and `odom_relay.py` bridges it
(`deploy/jetson02/README.md`).

---

## 0. Preflight (2 min, do not skip)

```bash
ros2 topic echo /rob_2/firmware/battery_averaged --once   # want > 11 V
systemctl is-active lidar lidar-tf leo-nav-bridge         # want active
systemctl is-active leo-nav                               # want inactive
```

Battery below ~11 V is the most common cause of a session that looks like a
software fault: telemetry drops, things close, and nothing in the logs says
"battery". Charge it before debugging anything.

## 1. Camera — terminal 1

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15
```

Keep 640×480×15. Higher resolution is the documented route to saturating the
Jetson↔SBC link and freezing firmware telemetry.

## 2. Stack — terminal 2

From `~/leo_sensor_ws`. Pick the line that matches how you will drive.

**Driving from the browser** — nothing on the Jetson may command the rover, or
you get two owners of the firmware command topic:

```bash
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false start_safety:=false \
  publish_lidar_tf:=false publish_odom_tf:=false publish_camera_tf:=true \
  start_sensor_fusion:=true start_slam:=true
```

**Driving from the keyboard through the gate** — `start_safety:=true`:

```bash
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false start_safety:=true \
  publish_lidar_tf:=false publish_odom_tf:=false publish_camera_tf:=true \
  start_sensor_fusion:=true start_slam:=true
```

With `start_safety:=true`, verify both before touching anything:

```bash
ros2 topic info /collision_monitor/footprint    # want 1 publisher, 1 subscription
ros2 topic info /cmd_vel                        # want exactly ONE publisher
```

An empty approach polygon means collision avoidance is silently checking
nothing — Humble approach polygons have no static-points parameter, the
polygon comes from `footprint_publisher.py`. A second `/cmd_vel` publisher is
someone else's teleop; find it before you drive.

## 2b. ArUco detector — OPTIONAL, and you probably want to skip it

**Detection can be done offline from the bag**, and offline is the better
choice — see "ArUco: live or offline" below. Run it live only if you want to
see markers appear in RViz during the drive. If you do, start it **after** the
stack is up.

```bash
mkdir -p ~/leo_nav2_ws/runs/current
ros2 run leo_nav2_exploration aruco_detector --ros-args \
  -p image_topic:=/rob_2/camera/color/image_raw \
  -p camera_info_topic:=/rob_2/camera/color/camera_info \
  -p dictionary:=DICT_4X4_50 -p marker_length:=0.08 \
  -p allowed_ids:="[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]" \
  -p frame_is_optical:=true -p rate_limit_hz:=5.0 \
  -p registry_file:="$HOME/leo_nav2_ws/runs/current/aruco_registry.json" \
  -p samples_file:="$HOME/leo_nav2_ws/runs/current/aruco_detections.csv"
```

`marker_length` is the side of the black square in metres and it is the one
number that can be wrong without anything erroring — the pose just lands short
or long along the view ray. 0.08 is what rover 2 was deployed with; measure
your actual plates. `deploy/jetson02/run_aruco.sh` is this same command frozen.

Check it is seeing things: `ros2 topic echo /aruco_markers --once`.

### ArUco: live or offline

Offline, on the laptop, once per bag:

```bash
MARKER_LENGTH=0.08 bash tools/offline_aruco.sh session1/bags/legA_2026... legA session1
```

Same detector, same registry file, same merge afterwards. It works because the
bag carries the three things detection needs: `/bag/color/compressed` (the
pictures), `/bag/color/camera_info` (K and D — without these there is no pose
at all), and `/tf` + `/tf_static`, whose `map -> odom` half came from the SLAM
that ran live, so offline marker poses land in the same frame as the saved map.

**Offline is better for three reasons.** `marker_length` is the one parameter
that can be wrong without anything erroring — the pose just lands short or long
along the view ray, and the merge inherits the error; offline you measure the
plates and re-run. A wrong `dictionary` detects nothing and looks exactly like
"no markers were in view". And it is one fewer thing to start, watch and
restart between legs — forgetting that restart is the trap two sections down.

**Live is better for one:** you find out in the lab whether the markers are
detectable at all, while you can still move them, add more, or improve the
lighting. If you have never run these plates before, run it live for the first
leg as a check, then rely on offline for the poses.

The one thing offline cannot recover is a frame you never recorded. At 2 Hz
with the detector's `min_hits: 3`, a marker must be in view ~1.5 s to be
confirmed — so **dwell on each marker for two or three seconds** when driving.

## 3. Recording — terminals 3 and 4

Same for both driving methods.

```bash
python3 tools/debug_color_throttle.py             # terminal 3, leave running
bash   tools/record_rover_bag.sh lab_teleop_1     # terminal 4, Ctrl+C to stop
```

**Use `full` if you want costmaps, plans or frontier goals afterwards** — see
"What you will and will not see" below. `lean` is ~25 MB/min, `full` ~48
MB/min, both at the throttle's default 2 Hz. Topics that don't exist in your
chosen mode are skipped with a note — with `start_safety:=false` the gate and
collision-monitor topics are simply absent.

### 5 Hz, 2 Hz — what those actually are

`debug_color_throttle.py` forwards **whole frames at the RealSense's own
resolution**, which step 1 sets to **640×480**. It does not resize anything;
the rate is the only thing it changes. Measured per frame: colour ~157 kB as
the driver's jpeg, depth ~190 kB as PNG-encoded 16-bit millimetres.

| rate | frames in 10 min | colour | + depth |
|---|---:|---:|---:|
| 2 Hz (default) | 1200 | 19 MB/min | 42 MB/min |
| 5 Hz | 3000 | 47 MB/min | 104 MB/min |

Add ~6 MB/min for everything else — lidar, TF, odom, IMU, cmd chain, ArUco.
2 Hz is choppy to watch and is fine for stills, the map timeline and the
costmap replay, which consumes ~5 Hz at most anyway. `HZ=5 bash ...` if the
presentation needs smoother video.

Ctrl+C, never `kill -9`: rosbag2 writes `metadata.yaml` on SIGINT only, and a
bag without it will not play.

### What you will and will not see

`safe_mapping.launch.py` runs SLAM, sensor fusion and the safety chain. It runs
**no Nav2** — no planner, no controller, no costmaps. So during the drive:

| | live | offline, from the bag |
|---|---|---|
| map growing, robot path | **yes** | yes |
| lidar, camera, odometry, battery | **yes** | yes |
| ArUco markers placed in the map | only if you ran 2b | **yes** — `tools/offline_aruco.sh` |
| local + global costmaps | no | **yes** — `scripts/drive_replay/` |
| Nav2 plans, frontier goals, goal allocation | no | **yes** — same pipeline |

The offline half is `scripts/drive_replay/replay_drive_wsl.sh`: it plays the
bag through the real-rover Nav2 bundle in shadow mode — scan filter →
slam_toolbox → costmaps → NavFn → RPP → explore_lite — so the frontier goals
and plans are computed against what the robot actually saw, and its `/cmd_vel`
lands on `/cmd_vel_shadow` next to what you actually drove. That is where the
costmap films come from.

It needs depth, so **record with `full`** if you want any of the bottom three
rows. It also reads fixed topic names (`/bag/color/compressed`,
`/bag/depth/compressed`, `/rob_4/camera/depth/camera_info`), which is exactly
why `debug_color_throttle.py` publishes those names — but that pairing has not
been run end-to-end on a bag recorded this way. Record a two-minute test bag
early in the session and put it through `scripts/drive_replay/probe_bag.py`
before you rely on it.

---

## 4a. Drive — browser (the simple one)

The rover's own SBC serves the stock Leo Rover UI. Nothing to start; it is
already running as part of `leo_bringup` (which is also what runs
`web_video_server`, `rosbridge` and `rosapi`).

**Either** join the rover's own Wi-Fi — SSID `leo-rover5a7c`, password
`password` — and open `http://10.0.0.1/`. That is a different network from the
lab Wi-Fi you SSH over, so it wants a second device or a second adapter.

**Or stay on lab Wi-Fi and tunnel through the Jetson**, which already reaches
the SBC over `enP8p1s0` (that link is what `ping 10.0.0.1` from the Jetson
measures during a starvation check):

```bash
ssh -L 8081:10.0.0.1:80 -L 9091:10.0.0.1:9090 jetson-04@192.168.178.104
```

Leave that session open and browse **`http://localhost:8081`**. Both ports are
needed: 80 serves the page, 9090 is the rosbridge the joystick and the camera
widget talk to. If the joystick moves but the rover does not, 9091 is not
forwarded or the UI has the rosbridge host hard-coded to `10.0.0.1` — in that
case fall back to joining the rover AP. *Not yet tried in the lab; the AP route
is the one that is known to work.*

Then drive with the on-screen joystick.

Two things to know, both of them documented failures on this robot:

* **Collapse or close the camera panel when you are not looking at it.** The
  UI subscribes to camera topics through its own rosbridge on
  `10.0.0.1:9090`, and an open tab is enough to drag image streams across the
  Jetson↔SBC link. Idle traffic is tens of KB/s; the firmware-starvation
  incident was ~1.4 MB/s, and it presents as dead hardware. Closing the tab is
  the first thing to try whenever firmware telemetry freezes.
* **The joystick goes straight to the firmware.** It does not pass the safety
  gate or the collision monitor — there is nothing between the browser and the
  wheels. That is why step 2 says `start_safety:=false` for this path: not
  because it is safer, but because otherwise two things are commanding the
  robot and neither wins predictably. You are the safety system here; stay
  within arm's reach.

The map still builds and the bag still records — SLAM and sensor fusion are up
regardless of who is driving.

## 4b. Drive — keyboard, through the safety chain

```bash
python3 tools/rover_teleop.py       # terminal 5, on the rover
```

`W`/`S` forward/back, `A`/`D` turn, `SPACE` stop, `-`/`=` scale, `Q` quit.
It publishes on `/cmd_vel_request`, the head of the chain:

```text
teleop / explorer   ->  /cmd_vel_request
safety_command_gate ->  /cmd_vel_raw
collision_monitor   ->  /cmd_vel
firmware_relay      ->  /rob_2/cmd_vel
```

Three things that will look like bugs and are not:

* **It's slow.** The gate caps 0.10 m/s and 0.30 rad/s whatever you request.
  Relaunch step 2 with `maximum_linear_speed:=0.15` if you need more.
* **`S` does nothing.** `maximum_reverse_speed` is 0.0 by default; reverse is
  blocked. Pass `maximum_reverse_speed:=0.05` — it also wants 0.75 m of rear
  clearance.
* **It stops in a corner.** Nose-first into a pocket gives <5% valid depth,
  the depth node stops publishing, the gate closes. Back out facing open
  space; it recovers immediately.

---

## 5. Close the leg — while SLAM is still alive

Ctrl+C the bag first, then:

```bash
bash tools/finish_run.sh legA          # or legB, legC...
```

That saves the map, serialises the pose graph, copies the ArUco registry under
the name the merger globs for, and moves the bag in beside them — all into
`~/leo_runs/session1/`. Do it **before** stopping the stack: `map_saver_cli`
and the serialise service both need SLAM alive.

---

## 6. One robot, two legs, one merged map

This is the supported path, and the one chosen for the lab day (2026-08-24)
over the live aligner. Two legs driven with the same robot, with **SLAM
restarted between them**, are exactly two rovers with an unknown relative pose:
each leg's map frame is anchored at that leg's own start, and nothing in the
merge knows it was one robot twice.

1. Drive leg A in one part of the building. `bash tools/finish_run.sh legA`.
2. **Restart the stack** (terminal 2) — and the ArUco detector too, if you ran
   it live. Then move the robot and restart the bag under `legB`.
3. Drive leg B. `bash tools/finish_run.sh legB`.

> **If you ran the detector live: restart it too, not just SLAM.** (Detecting
> offline sidesteps this entirely — one bag, one registry, by construction.)
> The registry accumulates marker
> positions *in the map frame*. Restarting SLAM moves the map origin to leg B's
> start; a detector left running keeps appending to the same registry, so leg
> A's markers stay in it at leg A's coordinates while leg B's arrive in leg B's.
> The merge then fits a transform to a registry that is half in one frame and
> half in another, and it will produce a confident, wrong answer rather than an
> error. `finish_run.sh legA` copies the registry first, so restarting the
> detector after it costs you nothing.
4. On the laptop:

```bash
scp -r jetson-04@192.168.178.104:~/leo_runs/session1 .
python3 scripts/align_registries_offline.py session1 --refine
```

It prints the recovered legB→legA transform, a leave-one-out table so one bad
landmark can be spotted and dropped with `--exclude <id>`, and writes the fused
map plus a PNG. About a second, re-runnable as often as you like.

**The one requirement:** both legs must confirm at least **2 of the same marker
ids** — 3 or more for the leave-one-out check to mean anything. Put markers in
the overlap region and drive both legs past them. `finish_run.sh` warns when a
leg confirms fewer than 2.

### What this gives you for the presentation, and what it does not

It gives you the payoff frame: two independently-built maps landing on each
other under a transform recovered from nothing but shared markers, residuals
printed. That is the claim, and it is measured.

It does **not** replay as a live merge. Nothing here plays two real bags
simultaneously into a live merger — `scripts/drive_replay/` replays one bag,
and `render_timelapse.py` renders the three-panel merge film only from the
`.npz` snapshots `merge_timelapse_recorder.py` writes during a *live* two-robot
run. The honest cut from two real bags is: leg A's map growing (bag replay in
RViz), leg B's map growing, then the merged result. The true "they snap
together at the moment of sighting" animation from two real bags is a script
that does not exist yet.

## Two things that must not happen

* **Never bag `/cmd_vel_raw`.** The gate audits its subscribers and closes on
  an unexpected one (`unexpected raw command consumers: rosbag2_recorder`).
  Recording it stops the robot; `ros2 topic echo` on it does the same.
  `record_rover_bag.sh` excludes it by construction.
* **Never publish keyboard teleop to `/cmd_vel`** while the safety stack is
  up. It bypasses the collision monitor and trips the same audit.
  `rover_teleop.py` refuses without `--unsafe`, and `--unsafe` is only correct
  with `start_safety:=false`.

## Afterwards, on the laptop

```bash
python3 tools/render_labeled_debug_video.py          # edit BAG/OUT at the top
wsl -d Ubuntu -- bash scripts/drive_replay/process_drive_wsl.sh <bag>
```

`tools/README.md` has the per-topic size breakdown if a bag comes back bigger
than expected; `docs/ROVER_FAILURE_RUNBOOK.md` has every live failure hit so
far and its quick fix.
