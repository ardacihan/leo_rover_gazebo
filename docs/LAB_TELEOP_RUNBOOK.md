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

## 3. Recording — terminals 3 and 4

Same for both driving methods.

```bash
python3 tools/debug_color_throttle.py             # terminal 3, leave running
bash   tools/record_rover_bag.sh lab_teleop_1     # terminal 4, Ctrl+C to stop
```

~20 MB/min. Add `full` as a second argument for depth (~45 MB/min) only if you
will replay the camera costmap layer. Topics that don't exist in your chosen
mode are skipped with a note — with `start_safety:=false` the gate and
collision-monitor topics are simply absent.

Ctrl+C, never `kill -9`: rosbag2 writes `metadata.yaml` on SIGINT only, and a
bag without it will not play.

---

## 4a. Drive — browser (the simple one)

The rover's own SBC serves the stock Leo Rover UI. Nothing to start; it is
already running as part of `leo_bringup` (which is also what runs
`web_video_server`, `rosbridge` and `rosapi`).

1. Join the rover's own Wi-Fi — SSID `leo-rover5a7c`, password `password`.
   This is a different network from the lab Wi-Fi you SSH to the Jetson over,
   so use a second device, or a second Wi-Fi adapter, or hop back and forth.
2. Open **`http://10.0.0.1/`**.
3. Drive with the on-screen joystick.

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

## 5. Artifacts — while SLAM is still alive

```bash
ros2 service call /save_mapping_artifacts std_srvs/srv/Trigger '{}'   # -> ~/leo_maps
ros2 run nav2_map_server map_saver_cli -f ~/leo_maps/lab_teleop_1
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph '{filename: /home/<user>/leo_maps/lab_teleop_1}'
```

Do this **before** stopping the stack. Then Ctrl+C the bag, then the rest, then
`scp` the bag and `~/leo_maps` off.

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
