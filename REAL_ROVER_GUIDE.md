# Real Leo Rover Field Guide

This is the operational runbook for connecting to the lab's physical Leo
Rovers, validating their sensors, running SLAM, and performing carefully
bounded motion tests. It combines the original lab tutorial with observations
made on the actual robots on 2026-08-05.

The observations in the per-robot sections are time-sensitive. Hardware is
moved between robots, batteries are changed, and colleagues run processes on
the shared accounts. Always repeat the preflight checks instead of assuming a
robot is still in the state described here.

## Non-negotiable motion rules

Before publishing any nonzero velocity:

1. Confirm a person is physically near the robot and the floor is clear.
2. Run `who` and inspect ROS/process ownership. These are shared lab machines.
3. Confirm fresh LIDAR/depth, odometry, battery, and TF data.
4. Confirm the intended command subscriber exists.
5. Confirm there is no competing `/cmd_vel` publisher such as teleop.
6. Start with a bounded test, normally 0.05-0.10 m/s for no more than 2 s.
7. Publish zero velocity repeatedly before and after the test.
8. Use a timeout and cleanup trap. Never leave an unbounded publisher running.
9. Save the map while the mapper is still alive.

Do not treat an advertised ROS topic as proof that data is live. ROS discovery
and the ROS CLI daemon can retain stale names. Verify fresh messages or count
messages using a short subscriber.

## Quick robot reference

| Jetson | SSH | ROS domain | Firmware namespace | LIDAR frame | Last verified caveat |
|---|---|---:|---|---|---|
| 1 | `jetson-01@192.168.178.101` | 1 | root `/firmware/*` | `laser` | C1 LIDAR works; RealSense presence changed; shared teleop/processes common |
| 4 | `jetson-04@192.168.178.104` | 4 | `/rob_2/firmware/*` | `laser_frame` | RealSense works; LIDAR/map were absent; host later went offline |
| 6 | unverified | unverified | unverified | none reported | User reports no LIDAR; do full discovery |

This table is a starting point, not a readiness result. Recheck every field at
the beginning of a session.

## Network and credentials

### Lab Wi-Fi (SSH)

| Field | Value |
|---|---|
| SSID | `FRITZ!Box 6690 FU` |
| Password | `66273063829385129175` |
| Subnet | `192.168.178.0/24` |

Expected Jetson addresses:

| Host | Address | Username/password |
|---|---|---|
| jetson-01 | `192.168.178.101` | `jetson-01` |
| jetson-02 | `192.168.178.102` | `jetson-02` |
| jetson-03 | `192.168.178.103` | `jetson-03` |
| jetson-04 | `192.168.178.104` | `jetson-04` |
| jetson-05 | `192.168.178.105` | `jetson-05` |

Jetson 6 is not in the original tutorial. A user reported that it does not
have a LIDAR, but its IP address, credentials, ROS domain, and current hardware
have not been verified. Do not assume it is `.106`.

Discover live documented hosts:

```bash
nmap -sn 192.168.178.0/24
```

Normal SSH does not need X11 forwarding:

```bash
ssh jetson-01@192.168.178.101
ssh jetson-04@192.168.178.104
```

Use `ssh -X` only when the client has an X server and a remote GUI is actually
needed. Prefer headless launch files over starting RViz on the Jetson.

### Rover access-point UI

The original tutorial also documents a rover access point:

| Field | Value |
|---|---|
| SSID | `leo-rover5a7c` |
| Password | `password` |
| UI | `http://10.0.0.1/` |

This is separate from the lab Wi-Fi used for Jetson SSH.

Treat all credentials in this file as lab-only credentials. Do not push them to
a public remote.

## Physical handling and power

- The rover power button is on its side.
- At startup, the green LED blinks. Wait until it stops blinking before SSH.
- Keep the rover away from table edges and do not run physical tests on a table.
- Keep food and drinks away from the robots.
- Do not force USB, camera, or power connectors.
- The tutorial describes the battery compartment behind three screws at the
  left rear of the rover.
- To charge, unplug the battery power cable and connect it to the adapter. A
  full charge takes approximately 4-5 hours.
- The original tutorial says to leave the rover powered on while charging.
- After changing a battery, wait for the Jetson and firmware to return, then
  repeat the complete preflight. A working SSH session alone is not sufficient.

If a rover unexpectedly disappears from SSH/ARP, stop all plans for motion and
check its power, battery, LED, and Wi-Fi physically. Do not assume it is only a
ROS discovery problem.

## First connection checklist

Run these before ROS commands:

```bash
hostnamectl
uptime
who
df -h /
free -h
lsusb
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
ps -eo user:12,pid,ppid,pgid,lstart,stat,args | \
  grep -E '[r]os2|[l]eo_|[r]plidar|[s]lam|[n]av2|[t]eleop|[r]viz'
```

Several people commonly use the same Unix account. An unfamiliar process is
not safe to kill just because its username matches yours. Check its parent,
process group, start time, terminal, and command line.

Use a dedicated workspace/directory for new code. A normal workflow is:

1. Develop and test in simulation on the workstation.
2. Commit/push through the team's Git remote.
3. Pull into a personal workspace on the Jetson.
4. Build only the required packages when possible.
5. Source the workspace after every build.

Example:

```bash
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws
colcon build --packages-select <package-name> --symlink-install
source install/setup.bash
```

Always source ROS and set the robot's domain explicitly in automation:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=<verified-domain>
```

Do not rely on `.bashrc`: noninteractive shells may not source it, and
`jetson-04` has accumulated multiple conflicting `ROS_DOMAIN_ID` exports.

## ROS graph and freshness checks

Useful discovery commands:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info -v /cmd_vel
ros2 topic info -v /scan
ros2 topic echo /scan --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /firmware/battery_averaged --once
```

Protect every diagnostic that could wait forever:

```bash
timeout 5s ros2 topic echo /scan --once
timeout 6s ros2 topic hz /scan
timeout 4s ros2 run tf2_ros tf2_echo odom base_footprint
```

The ROS CLI daemon showed stale nodes/topics during testing. A process list and
a fresh message subscriber are more trustworthy than `ros2 node list` alone.
If other operators are present, do not restart the shared CLI daemon without
coordination.

Recommended readiness criteria for an initial test:

- LIDAR: approximately 10 Hz and a substantial fraction of finite ranges.
- Wheel odometry: at least 10 Hz, zero velocity while the robot is at rest.
- IMU, when used: fresh and approximately 9.8 m/s^2 on the gravity axis.
- Battery: comfortably above the firmware cutoff, not merely above it.
- `/cmd_vel`: the intended subscriber is present and no competing publisher is
  active before the safety/control process starts.
- TF: one unambiguous chain from `odom` through the base to the scan frame.
- SLAM: `/map` has a live publisher and nonzero known/occupied cells.

## Battery and firmware behavior

The installed Leo configuration uses a minimum battery voltage of `10.0 V`:

```text
/opt/ros/humble/share/leo_bringup/config/firmware_diff_drive_params.yaml
/opt/ros/humble/share/leo_fw/data/default_firmware_params.yaml
```

Do not use `10.0 V` as an operating target. On `jetson-01`, a battery reading
near `10.35 V` coincided with firmware wheel/IMU publishers repeatedly
appearing and disappearing. After replacement, the battery was `12.12 V`, the
firmware and command path were stable, and bounded motion succeeded. For
extended autonomy, recharge or replace a battery that is near `10.5 V` rather
than waiting for the firmware cutoff.

After a battery/power cycle, SSH may return before the ROS sensor stack. Repeat
all freshness checks and relaunch only missing processes.

## Jetson 1 (`192.168.178.101`)

### Jetson 1 ROS environment

- ROS 2 Humble on Ubuntu 22.04/Jetson ARM64.
- `ROS_DOMAIN_ID=1` for the rover processes observed in this session.
- Firmware topics were unnamespaced, for example `/firmware/wheel_odom`.
- The firmware subscribed directly to `/cmd_vel` when healthy.
- The RPLIDAR C1 used `/dev/ttyUSB0`, `460800` baud, Standard scan mode, and
  published `/scan` with frame `laser`.
- RealSense availability changed during the day. It was enumerated and tested
  early in the session, but after a later reboot `lsusb` showed no RealSense,
  matching the operator report that Rover 1 currently has no RealSense. Always
  run `lsusb`/`rs-enumerate-devices` rather than assuming it is installed.

### RPLIDAR C1

`jetson-01` has a workspace version of `rplidar_ros` with C1 launch files:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch rplidar_ros rplidar_c1_launch.py
```

`view_rplidar_c1_launch.py` also starts RViz. Avoid it for headless operation
unless someone needs the on-Jetson GUI.

Equivalent direct driver parameters:

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p channel_type:=serial \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=460800 \
  -p frame_id:=laser \
  -p inverted:=false \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
```

Never start a second driver on the same serial device. Check first:

```bash
fuser /dev/ttyUSB0
ps -ef | grep '[r]plidar_node'
```

One measured 10-second LIDAR sample produced about 10.0 Hz, 720 ranges per
scan, and about 82% valid readings.

### Rover sensor bringup

The standard bringup used during the successful test was:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch leo_bringup leo_bringup.launch.xml wheel_odom:=/wheel_odom
```

Expected processed topics include:

```text
/wheel_odom
/merged_odom
/imu/data
/firmware/battery_averaged
/joint_states
/tf
/tf_static
```

Known non-blocking problems on this machine:

- `leo_camera::CameraNode` can fail with "Could not find requested resource in
  ament index". This is the built-in CSI camera path, not the RealSense driver.
- `firmware_parameter_bridge` may report "Firmware parameter service not
  active" while telemetry and `/cmd_vel` still work.
- `web_video_server` or rosbridge can conflict with already-running ports.

Do not ignore missing motion telemetry just because the processes exist. During
a controller dropout, `odom_filter` continued publishing filtered odometry even
though fresh wheel odometry and IMU input had stopped.

### TF caveat

Different temporary sessions published different LIDAR transforms, including
`base_footprint -> laser` Z values of `0.15` and approximately `0.398` m. The
repository simulation model is also not a physical calibration. Inspect the
actual mount and the current TF tree before mapping:

```bash
timeout 5s ros2 run tf2_ros tf2_echo base_footprint laser
```

Ensure exactly one publisher owns the LIDAR transform. For planar SLAM, X/Y and
yaw errors are especially damaging. Do not add a second static transform merely
because `tf2_echo` was slow to respond.

### Hardware SLAM package added to this repository

This repository now contains `src/leo_rover_real_bringup`, a small package that
does not depend on Gazebo. It provides:

- `config/slam_params.yaml`: wall time, real frames/topics, 5 cm resolution,
  and 5 cm/radian scan insertion thresholds.
- `launch/slam.launch.py`: configurable static LIDAR transform plus asynchronous
  SLAM Toolbox.
- `launch/safe_mapping.launch.py`: wheel-only odometry TF, SLAM Toolbox, a
  fail-closed command gate, Nav2 Collision Monitor, and an optional bounded
  explorer.
- `scripts/wheel_odom_tf.py`: integrates wheel twist without the stationary IMU
  yaw drift seen on Rover 1.
- `scripts/safety_command_gate.py`: permits only fresh, capped commands when
  LIDAR, wheel odometry, and battery telemetry are healthy. Reverse is limited
  to straight, slow motion with an independently checked rear corridor.
- `scripts/safe_room_explorer.py`: a bounded room explorer with 180-second and
  12-meter hard ceilings, planned turns, swept-corner checks, and guarded
  reverse recovery. It remains capped at 0.10 m/s and 0.30 rad/s.

Build and launch:

```bash
cd <repo-workspace>
colcon build --packages-select leo_rover_real_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch leo_rover_real_bringup slam.launch.py \
  lidar_x:=<measured-x> lidar_y:=<measured-y> \
  lidar_z:=<measured-z> lidar_yaw:=<measured-yaw>
```

A copy was also deployed to `~/codex_ws` on `jetson-01` during testing.

For the safe hardware path, the standard odometry filter must not also publish
`odom -> base_footprint`. On 2026-08-06 its IMU-integrated yaw drifted by more
than 7 degrees while Rover 1 was stationary, producing misleading scan poses.
Start bringup with its TF disabled, then start safe mapping without the
explorer:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch leo_bringup leo_bringup.launch.xml \
  wheel_odom:=/wheel_odom publish_odom_tf:=false

source ~/codex_ws/install/setup.bash
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false
```

Before motion, verify exactly this command path:

```text
bounded explorer       -> /cmd_vel_request
safety_command_gate    -> /cmd_vel_raw
collision_monitor      -> /cmd_vel
Leo firmware           <- /cmd_vel
```

Only after the full readiness gate and physical-operator confirmation, start a
single bounded segment:

```bash
ros2 run leo_rover_real_bringup safe_room_explorer.py --ros-args \
  -p run_duration:=30.0 -p max_distance:=2.0
```

The older repository file `src/leo_rover_gazebo/launch/slam.launch.py` is
simulation-only: it uses `/leo1/scan`, simulation frames/time, and a hard-coded
container path. Do not use it on a physical rover.

### Successful bounded SLAM test

The verified sequence on `jetson-01` was:

1. Replace low battery; confirm approximately 12.1 V.
2. Start C1 LIDAR and rover bringup.
3. Start `leo_rover_real_bringup` SLAM.
4. Confirm fresh odometry, map, one firmware command subscriber, and no command
   publisher.
5. Publish 0.08 m/s forward for 2 s, then zero repeatedly.
6. Observe 0.1533 m odometry translation and 4,830 additional known map cells.
7. Publish -0.08 m/s for 2 s to return, then zero repeatedly.
8. Observe 0.1540 m return motion and final velocities of zero.
9. Save the map while SLAM is still alive.

The resulting test map was 124 x 178 cells at 0.05 m/cell. Local artifacts are
under `artifacts/jetson01_slam/`; the post-motion map is `post_motion.pgm`,
`post_motion.yaml`, and `post_motion.png`.

### Collision-aware room mapping run (2026-08-06)

A later Rover 1 session completed the staged collision-aware test with a fresh
battery and a person beside the robot:

- Preflight battery was 12.06-12.10 V; the C1 LIDAR ran at 10.05 Hz and wheel
  odometry at 20.98 Hz.
- The only command path was `/cmd_vel_request` -> `safety_command_gate` ->
  `/cmd_vel_raw` -> `collision_monitor` -> `/cmd_vel` -> firmware.
- A 0.05 m/s, two-second bounded test traveled 0.0894 m, never exceeded the
  requested speed, and ended with zero measured and commanded velocity.
- A five-second autonomous stage traveled 0.139 m and stopped at its deadline.
- Two 30-second autonomous segments at 0.06 m/s traveled 1.727 m and 1.383 m.
  No reverse motion or collision occurred. During the second segment,
  Collision Monitor reduced the final command to 0.021 m/s when its slowdown
  zone detected nearby geometry.
- Averaged battery telemetry stayed near 12 V. One instantaneous sample under
  motion was 11.71 V, and the stopped reading was 11.93 V.
- Repeated final zero requests were sent, wheel odometry reported zero twist,
  and all session-owned mapper, LIDAR, and rover bringup processes were stopped
  after the map was saved.

The final saved map is 188 x 230 cells at 0.05 m/cell (approximately
9.4 x 11.5 m). Rover-side files are
`~/maps/leo_room_20260806_final.{pgm,yaml}`. Local copies and a PNG preview are:

```text
artifacts/jetson01_slam/leo_room_20260806_final.pgm
artifacts/jetson01_slam/leo_room_20260806_final.yaml
artifacts/jetson01_slam/leo_room_20260806_final.png
```

The first explorer attempt exposed an idle-output handshake detail: Humble's
Collision Monitor can remain silent while its input is zero. The explorer now
requires LIDAR, odometry, and battery before its first actionable request, then
requires a fresh Collision Monitor output. The fail-closed gate, command
timeout, speed caps, and final-zero behavior remain in force.

### Fuller-map attempt and recovery redesign (2026-08-06)

The later aggressive attempt accumulated about 9.7 m across restarts and saved
a 214 x 288-cell checkpoint plus its resumable pose graph. The map has broader
coverage but visible distortion, so it is a recovery checkpoint rather than a
clean navigation map:

```text
artifacts/jetson01_slam/leo_room_fuller_20260806_final.{pgm,yaml,png}
artifacts/jetson01_slam/leo_room_fuller_20260806_final.{posegraph,data}
```

That attempt exposed a forward-only recovery defect: near a wall the explorer
kept requesting an in-place turn instead of backing out, and the wheels touched
the wall. Motion was stopped with repeated zero commands, the map was saved,
and all physical command publishers were shut down.

The replacement logic now:

- turns at a persistent dense front obstacle instead of terminating;
- requires measured yaw progress before considering a turn complete;
- treats a Collision Monitor-held forward command as a cue to turn;
- escalates a blocked/boxed turn to at most 0.04 m/s of straight reverse, for
  at most 0.35 m per attempt, only with at least 0.75 m rear clearance;
- aborts reverse immediately if the rear corridor closes;
- uses a direction-aware time-to-collision footprint rather than a static zone
  that also blocks motion away from a wall;
- enforces a 10.50 V battery floor.

Six pure decision tests and two isolated ROS controller scenarios passed: a
boxed front selected straight reverse, and simulated forward/turn suppression
escalated to reverse; both stopped when a rear obstacle appeared. The final
on-device Collision Monitor dynamics probe was interrupted when Rover 1 became
unreachable. Before physical reuse, reconnect/recharge, ensure no
`/codex_cm/*` probe remains, rebuild the workspace, and repeat a short staged
test with the rover repositioned away from the wall.

### Saving a map

Use the standard Nav2 map saver while `/map` is live:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/room_map \
  --ros-args -p save_map_timeout:=20.0
```

Verify both files:

```bash
ls -lh ~/maps/room_map.pgm ~/maps/room_map.yaml
cat ~/maps/room_map.yaml
```

One detached mapper was externally terminated after map growth but before the
save call. A subsequent self-contained session saved successfully. Keep the
mapper and map saver in the same managed session, or save immediately before
cleanup.

### Collision avoidance status on Jetson 1

`nav2_collision_monitor` is installed. A conservative LIDAR configuration and
upstream safety gate were deployed and tested on isolated command topics on
2026-08-06. At the earlier check on 2026-08-05, another operator's keyboard
teleop was publishing directly to `/cmd_vel`; command ownership must still be
rechecked every session.

For future autonomous testing, use this command topology:

```text
bounded explorer or Nav2 controller -> /cmd_vel_request
fail-closed safety command gate      -> /cmd_vel_raw
Nav2 Collision Monitor               -> /cmd_vel
Leo firmware                          <- /cmd_vel
```

The collision monitor must be the only normal `/cmd_vel` publisher. A teleop or
controller publishing directly to `/cmd_vel` bypasses the monitor and must be
stopped or routed through a proper priority/safety mux first.

Important Humble behavior verified on 2026-08-06: Collision Monitor alone did
not fail closed when its configured scan topic was absent; it passed a nonzero
isolated input through unchanged. The separate `safety_command_gate` is
therefore required. An isolated repeat test confirmed that the gate output zero
with stale scan data, capped a `0.50 m/s, 1.0 rad/s` request to
`0.10 m/s, 0.30 rad/s`. A later isolated test confirmed that clear-corridor
reverse passes at `-0.04 m/s` and a blocked rear produces zero. None of those
test topics were connected to the firmware.

During the same session, battery voltage fell from about 10.96 V to 10.68 V
while stationary and firmware telemetry began dropping out. The gate and
explorer stopped as designed. Replace or recharge the battery before any
physical autonomous test; do not lower the safety package's 10.50 V floor to
work around dropout. A retrying `leo-ros.service` was also found sourcing a workspace
without `leo_real` and requesting `enable_supervisor:=true`; it was stopped for
the session but remains a boot-time maintenance issue.

Recommended initial limits for a staged test:

- Linear speed: at most 0.10-0.12 m/s.
- Angular speed: at most 0.30-0.40 rad/s.
- Stop zone: at least 0.45 m forward, adjusted for the physical footprint.
- Slowdown zone: approximately 0.70-0.80 m.
- Sensor timeout: no more than 0.5 s; stale LIDAR must produce zero velocity.
- Total first autonomous run: 15-30 s with a person next to the robot.

These are conservative starting values, not a completed validation. Test the
stop layer with zero/very-low-speed commands before room exploration.

## Jetson 4 (`192.168.178.104`)

### Jetson 4 ROS environment

- `ROS_DOMAIN_ID=4` for the installed robot processes.
- The physical firmware was namespaced as `/rob_2`, not `/rob_4`.
- Workspace: `~/ros_ws`.
- A RealSense D456 was present and streaming.
- A CP210x serial adapter was present at `/dev/ttyUSB0`, consistent with a C1
  LIDAR connection, but the LIDAR stream was not live during the audit.

The boot-time stack observed on 2026-08-05 included:

```text
ros2 launch leo_real leo_real.launch.py enable_supervisor:=false
~/ros_ws/install/leo_nav_bridge/bin/bridge
static_transform_publisher base_link -> laser_frame
ros2 launch leo_nav navigation.launch.xml
slam_toolbox async_slam_toolbox_node
navigation_container (planner/controller/BT/waypoint components)
```

The installed static transform command used:

```text
base_link -> laser_frame
x=0.0775, y=0, z=0.048, roll=0, pitch=0, yaw=3.14159
```

Verify it against the physical mount before reuse.

### Navigation bridge

`leo_nav_bridge` performed these fixed mappings:

```text
/rob_2/firmware/wheel_odom -> /wheel_odom and /merged_odom
/rob_2/firmware/wheel_odom -> odom -> base_footprint TF
/cmd_vel                   -> /rob_2/cmd_vel
```

Therefore, the root `/cmd_vel` on Jetson 4 controls firmware in the `/rob_2`
namespace. Do not infer the robot namespace from the Jetson number.

### RealSense observations

The aligned depth stream was verified at about 13.2 Hz:

```text
/camera/camera/aligned_depth_to_color/image_raw
sensor_msgs/msg/Image
640x480, 16UC1, frame camera_color_optical_frame
```

The `leo_real` stack also ran:

```text
/image_processor_rgb
/robot_supervisor_rgb
```

The image processor generates depth-based obstacle zones/distances. However,
the supervisor was launched with `enable_supervisor:=false`. Even while
disabled, it advertised a `/cmd_vel` publisher endpoint. The Nav2 controller
also advertised `/cmd_vel`, so the bridge saw two possible publishers.

Do not simply enable the supervisor for exploration. Its source includes
actions around 0.2 m/s and random turns up to approximately 1 rad/s, which are
too aggressive for an unvalidated first run. Resolve command ownership and
lower limits first.

### Installed Nav2 configuration

The robot-specific `leo_nav` configuration was much better suited to hardware
than this repository's simulation Nav2 file:

- `use_sim_time: false`.
- `map`, `odom`, and `base_footprint` frames.
- 44 x 44 cm footprint (`+/-0.22 m`).
- LIDAR obstacle layer in the local costmap.
- 0.27 m inflation radius.
- A* NavFn global planner.
- DWB local controller.

Important caveats:

- Configured maximum linear speed was `0.26 m/s`; lower it before testing.
- Configured maximum angular speed was `0.85 rad/s`; lower it before testing.
- SLAM scan insertion thresholds were 0.5 m and 0.5 rad, too coarse for tiny
  bringup motions.
- The local costmap consumed only `/scan`; RealSense depth was not fused into
  the costmap.

For LIDAR plus RealSense operation, use a deliberate fusion architecture such
as depth-image-to-laserscan/pointcloud in the costmap plus a single collision
monitor output. This requires a verified base-to-camera extrinsic transform.
Do not claim camera fusion merely because depth topics exist.

### Failed readiness state and outage

During the audit, Jetson 4 had:

- Fresh odometry at about 20.2 Hz.
- Fresh aligned RealSense depth at about 13.2 Hz.
- No `/scan` messages.
- No live `/map` messages.
- No live local costmap messages.
- Two `/cmd_vel` publishers (controller and disabled supervisor) feeding one
  bridge subscriber.

This was not safe for motion.

The `/opt/ros/humble` RPLIDAR installation did not include a C1 launch file,
only A/S/T-series launch files. A direct C1-parameter driver start was attempted
using `/dev/ttyUSB0`, 460800 baud, and `laser_frame`; immediately afterward the
Jetson stopped responding to SSH/ARP. No motion command was sent. The host did
not return after a full boot wait.

This timing is an observation, not proof that the driver caused the outage.
Before the next attempt, physically check power/battery, restore SSH, inspect
boot logs, check `/dev/ttyUSB0` ownership, and confirm the actual LIDAR model.

## Jetson 6

Only one fact has been reported: it currently has no LIDAR. Nothing else was
verified. Traditional 2D LIDAR SLAM cannot be assumed to work there.

Possible camera-only mapping would require all of the following to be verified:

- Actual IP, credentials, and ROS domain.
- A working depth camera and fresh calibrated depth topics.
- Base-to-camera TF/extrinsic calibration.
- A tested depth-to-scan, pointcloud, or visual SLAM pipeline.
- Collision monitoring that fails closed when camera data is stale.

Do not improvise autonomous motion on Jetson 6 merely by publishing depth data
as a scan.

## RealSense D456 bringup and testing

Enumerate before launching:

```bash
lsusb | grep -i realsense
rs-enumerate-devices -s
```

Typical ROS launch:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=<robot-domain>
ros2 launch realsense2_camera rs_launch.py
```

Typical topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
/camera/camera/aligned_depth_to_color/image_raw
```

One D456 test produced valid 1280 x 720 RGB and 848 x 480 depth frames. During
that test, killing only the `ros2 launch` parent left camera child processes
alive. A second launch then contended for the USB device and produced V4L2
errors. Always terminate the entire process group and confirm:

```bash
ps -ef | grep '[r]ealsense2_camera_node'
```

The built-in Leo CSI camera topics (for example `/camera/image_raw`) are not the
same as RealSense topics under `/camera/camera/...`.

## Managed process pattern

For a temporary remote launch, give it a process group, record it, and redirect
logs:

```bash
log=/tmp/my_test.log
nohup setsid bash -lc '
  source /opt/ros/humble/setup.bash
  source ~/my_ws/install/setup.bash
  export ROS_DOMAIN_ID=1
  exec ros2 launch my_package my.launch.py
' >"$log" 2>&1 < /dev/null &

pid=$!
echo "$pid" >/tmp/my_test.pid
```

Cleanup only the process group you created:

```bash
pid=$(cat /tmp/my_test.pid)
kill -INT -- -"$pid"
sleep 5
kill -TERM -- -"$pid" 2>/dev/null || true
```

Check the process group before killing it:

```bash
ps -eo user:12,pid,ppid,pgid,lstart,stat,args | grep " $pid "
```

Launch parents were sometimes externally terminated while children remained.
Inspect PGIDs and child processes rather than assuming a missing parent means
the whole stack stopped. Never use broad `pkill` patterns on a shared robot.

For the most reliable map test, keep bringup, SLAM, bounded motion, map save,
and cleanup in one shell with a trap. The trap must publish zero velocity before
terminating control nodes.

## Bounded motion template

Do not paste this until the readiness gate passes. The idea is a finite command
followed by repeated zeros, not an indefinitely running publisher:

```bash
# Example only: approximately 10-16 cm on a healthy rover.
timeout 3s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.06}, angular: {z: 0.0}}'

# Always stop explicitly; publish more than one zero.
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

For automated tests, use a node with a hard wall-clock deadline and a `finally`
block that publishes zero for at least one second. Add an independent watchdog
when testing longer behaviors.

## Full next-session procedure

1. Connect to lab Wi-Fi and probe the documented Jetson IPs.
2. SSH to one robot; record hostname, uptime, `who`, and current processes.
3. Enumerate USB hardware; do not infer sensor presence from old docs.
4. Source ROS/workspaces and explicitly set the verified ROS domain.
5. Inspect namespaces and command bridges; Jetson number may not equal robot
   namespace.
6. Verify battery comfortably above cutoff.
7. Verify fresh sensor rates and sample validity for 10 seconds.
8. Verify the complete, single-owner TF chain.
9. Verify no teleop/controller/supervisor is competing for `/cmd_vel`.
10. Start missing bringup components only; do not duplicate serial drivers.
11. Start SLAM while stationary and inspect real occupancy-grid contents.
12. Insert collision monitoring so autonomous raw commands cannot bypass it.
13. Validate fail-closed behavior: stale scan/depth must output zero.
14. Run a 1-2 second low-speed motion and compare odometry/map before/after.
15. Only then consider a 15-30 second bounded exploration with a physical
    operator and an explicit maximum speed/distance/time.
16. Save the map while the mapper is alive.
17. Publish repeated zeros, verify measured zero velocity, stop only your
    process groups, and leave colleagues' sessions untouched.

## Simulation versus physical robot

| | Simulation | Physical rover |
|---|---|---|
| Time | `use_sim_time: true` | `use_sim_time: false` |
| Frames/topics | Often namespaced (`leo1/*`) | Must be discovered on each robot |
| Sensors | Gazebo plugins | USB/firmware hardware with possible dropouts |
| Motion safety | No physical risk | Requires operator, clear floor, bounded commands |
| SLAM launch | Existing Gazebo package | `leo_rover_real_bringup` or installed robot stack |

Simulation configuration must not be deployed unchanged to hardware. The
existing Gazebo/Nav2 files contain simulated frames, topics, time, and higher
velocity limits.
