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

### Rover 4 LIDAR and camera mount geometry (2026-08-12)

The physical LIDAR scan plane is approximately 0.04 m above the rover top
plane. The rear-right mounted camera reaches approximately 0.20 m above the
rover top plane, so its top is about 0.16 m above the LIDAR scan plane. The
camera body may be above the beam, but its bracket, cable, base, or lower
housing can still intersect the horizontal scan.

### Rover 4 mount calibration, corrected (2026-08-13)

The 2026-08-12 entry above understated the self-return band and wrongly
suspected the LIDAR yaw. Both were re-measured over 200 stationary scans.

**The LIDAR yaw is correct.** The live transform is:

```text
base_footprint <- laser_frame: x=0.0775  y=0.04  z=0.2458  yaw=3.14159
```

The `y=0.04` is a correction applied later the same day; see "Rover 4 lidar
lateral offset" below. Everything in this section was measured while `y` was
still 0, which shifts the mast bearings quoted here but changes none of the
conclusions.

The camera mast only *appears* on the robot's left in raw scan angles because
the LIDAR is mounted yawed by pi. Rotating by that yaw puts the mast at
`-130 deg to -102 deg` in `base_footprint`, which matches the physical mount.
Never compare raw scan angles against robot-frame directions.

**The self-return band is wider than recorded.** Every persistent close return
fell between `+16 deg` and `+79 deg` in the laser frame at `0.023-0.174 m`,
including grazing returns at `2-5 cm`, which is below the sensor's own
`range_min` of 0.15 m. The old `+45..+82 deg` window left roughly half of them
unmasked, and every unmasked point sat inside Collision Monitor's 0.31 m
approach circle, which would have vetoed motion permanently.

The production `scan_fusion.py` primary mask is deliberately bounded to the
measured raw-laser interval `+12..+83 deg` and ranges at or below 0.22 m. A
farther point at the same bearing is retained. A second base-frame backstop
drops only points that land within 0.22 m of the base origin; it catches an
intermittent bracket return measured at raw `-142..-140 deg`, about 0.033 m,
without hiding external obstacles. The older gate's radial filter remains only
as a legacy/raw-scan fallback; `safe_mapping.launch.py` disables it. Both
collision avoidance and SLAM now consume base-frame fused topics rather than
`/scan_self_filtered` or raw `/scan`.

The mast fully occludes roughly `-130..-102 deg`, so that wedge is a permanent
blind spot. It lies outside the gate's rear corridor (`|y| <= 0.30 m`), so it
does not affect reverse checks.

**Sector logic must be frame-corrected.** `safe_room_explorer.py` and
`safety_command_gate.py` computed sectors directly from raw scan angles, so on
this rover every sector was reflected: "front" measured the physical rear, and
the gate's rear corridor measured the forward corridor. Both nodes now resolve
the mounting yaw from TF at runtime (`scan_yaw_offset`, NaN means auto) and
fail closed while the transform is unavailable.

Ground-truth check with the rover parked beside a shelf, resolved into
`base_footprint`: FRONT 3.5 m, LEFT 3.0-3.6 m, RIGHT 0.54-0.91 m (the shelf),
REAR 1.5-1.6 m. The raw-angle numbers for the same instant claimed left 0.16 m
and right 1.9 m, i.e. left and right swapped.

### Rover 4 lidar lateral offset, and which unit owns the transform (2026-08-13)

`base_link -> laser_frame` is **not** published by `lidar.service`. A separate
unit, `lidar-tf.service`, owns it. Look there before concluding that a lidar
transform is unowned or that a launch file must publish it.

The stock unit hardcoded `--y 0`, but the lidar is physically **0.04 m to the
rover's left**. Every map built before 2026-08-13 therefore carries a 4 cm
lateral bias. Two independent measurements agree on the correction:

- An operator tape measurement of the mount.
- The lidar's own view of the camera mast. Under the `y=0` assumption the mast
  resolved to `y=-0.074`; with `y=0.04` it resolves to `y=-0.038`, against a
  tape measurement of the camera at `-0.040`.

The fix is a systemd drop-in, so it survives reboot and applies to anyone who
uses the boot stack without this repository's launch file:

```text
/etc/systemd/system/lidar-tf.service.d/override.conf
```

```ini
[Service]
ExecStart=
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0.0775 --y 0.04 --z 0.048 --roll 0 --pitch 0 --yaw 3.14159 --frame-id base_link --child-frame-id laser_frame"
```

Then `sudo systemctl daemon-reload && sudo systemctl restart lidar-tf`. Verify
with `systemctl show lidar-tf -p ExecStart` and `tf2_echo base_footprint
laser_frame`, which must report `[0.077, 0.040, 0.246]`.

Because the boot unit now publishes the correct transform, run
`safe_mapping.launch.py` with `publish_lidar_tf:=false`. Setting it true while
`lidar-tf.service` is active gives `laser_frame` two parents
(`base_link` and `base_footprint`) and the TF tree becomes ambiguous.

### Rover 4 lidar model: it is a C1, not an S3 (2026-08-13)

`lidar-tf.service` was described as "Static TF for RPLIDAR S3". That name is
wrong and misled model assumptions. The driver's own startup log settles it:

```bash
journalctl -u lidar.service | grep -E "scan mode|Firmware|Hardware"
```

```text
current scan mode: DenseBoost, sample rate: 5 Khz, max_distance: 40.0 m,
scan frequency: 10.0 Hz
Firmware Ver: 1.02   Hardware Rev: 18
```

**Sample rate is the discriminator.** An S3 samples at 32 kHz; this reports
5 kHz, which is C1-class, and 5 kHz at 10 Hz gives the ~510 points per scan
actually observed on `/scan`. The unit connects through a CP210x bridge
(`10c4:ea60`) symlinked to `/dev/lidar` by a udev rule, at 460800 baud.

Do not trust the advertised `range_max` of 40.0 m. That figure comes from the
scan-mode descriptor, not the optics; a C1 is a ~12 m sensor. Treat returns
beyond about 12 m as unreliable and do not size costmaps or filters from
`range_max`. The furthest genuine return measured in the lab was 7.46 m.

### Our own DDS traffic starves the rover's firmware (2026-08-13)

**Symptom.** Firmware telemetry (`/rob_2/firmware/*`) stops dead a minute or so
after the mapping stack starts. The `/rob_2/firmware` node stays on the graph,
its message counters freeze at an exact value, and `/firmware/boot` stops
answering. It looks exactly like dead hardware.

**It is not hardware.** Hours were lost to that assumption. Three battery packs
were swapped, and the rover's own computer was rebooted through
`/system/reboot`, all with no effect. What actually settled it was a controlled
experiment:

| Condition | Result |
|---|---|
| Rover idle, no stack running | 510 s, zero dropouts |
| Plus `imu_odometry.launch.py` (light) | still zero dropouts |
| Plus `safe_mapping.launch.py` (heavy) | dead in about 60 s |
| Stack stopped again | recovered on its own after 186 s, no power cycle |

**Cause.** The rover's onboard computer shares `ROS_DOMAIN_ID=4` with the
Jetson over the `enP8p1s0` Ethernet link, and the Jetson's DDS multicast route
points at that link:

```text
multicast 239.255.0.1 dev enP8p1s0 src 10.0.0.76
```

Measured on that interface:

```text
stack down :  51 KB/s to the rover
stack up   : 1396 KB/s to the rover      (27x)
ping RTT   : 0.26-0.53 ms idle -> 0.35-19.9 ms under load
```

The rover is a small SBC whose real job is talking to the LeoCore over serial.
Roughly 1.4 MB/s of camera, depth, scan, map and costmap traffic it never asked
for saturates it, its firmware node stops being scheduled, and telemetry stops.
Recovery does not need a power cycle, only silence.

**Diagnosing it again.** Watch the interface counters rather than guessing:

```bash
read -r r1 t1 < <(awk '/enP8p1s0/{gsub(/:/," ");print $2, $10}' /proc/net/dev)
sleep 10
read -r r2 t2 < <(awk '/enP8p1s0/{gsub(/:/," ");print $2, $10}' /proc/net/dev)
echo "to rover $(( (t2-t1)/10/1024 )) KB/s"
```

Anything far above the ~51 KB/s idle baseline is the fault reappearing. A ping
to `10.0.0.1` climbing from sub-millisecond into double digits says the same
thing.

**Fixes, cheapest first.**

1. Find what is actually pulling the bulk across. The rover serves a web UI
   that subscribes to camera topics through its own rosbridge on
   `10.0.0.1:9090`, so an open browser tab on that UI is enough to drag image
   streams over the link. Close it and re-measure before changing anything.
2. Keep the heavy topics off the wire. Everything except the firmware topics
   and `/cmd_vel` is consumed entirely on the Jetson, so the traffic has no
   reason to leave the machine.
3. The robust fix is to separate the domains: put the mapping stack on its own
   `ROS_DOMAIN_ID` and bridge only the handful of topics that must cross
   (`/rob_2/firmware/wheel_odom`, `/rob_2/firmware/imu`,
   `/rob_2/firmware/battery_averaged`, `/rob_2/cmd_vel`) with `domain_bridge`.
   Bulk traffic then cannot reach the rover at all.

Do not raise speeds, swap batteries or reflash anything in response to this
symptom. Measure the link first.

### Rover 4 verified command path (2026-08-13)

Rover 4 starts its stack from systemd at boot, so a fresh session already has
SLAM and Nav2 running. `safe_mapping.launch.py` used to start a second
slam_toolbox, a second `odom -> base_footprint` publisher and a conflicting
LIDAR transform on top of it. The launch file now takes `publish_lidar_tf`,
`publish_odom_tf`, `publish_camera_tf` and `start_slam` so it composes with
whatever already owns each piece.

To take ownership of `/cmd_vel` on Rover 4:

```bash
sudo systemctl stop leo-nav          # controller_server + coarse boot SLAM
ros2 daemon stop && ros2 daemon start  # the CLI cache lies after this
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false publish_lidar_tf:=false \
  publish_odom_tf:=false publish_camera_tf:=true start_slam:=true
```

`lidar.service`, `lidar-tf.service`, `leo-ros.service` (RealSense) and
`leo-nav-bridge.service` must keep running: they own the scan, corrected LIDAR
transform, camera, odometry path, and the `/cmd_vel -> /rob_2/cmd_vel` firmware
hop. The verified `lidar-tf.service` transform is based at `base_link` with
`x=0.0775`, `y=0.04`, `z=0.048`, and yaw pi; through the URDF this resolves to
the documented `base_footprint <- laser_frame` height. Keep
`publish_lidar_tf:=false` while that service is active.

Restore the boot configuration with `sudo systemctl start leo-nav` after
killing the launch.

Verified live on 2026-08-13: `/scan` 10.0 Hz,
`/camera/scan_collision` and `/camera/scan_slam` about 13.0 Hz, both fused
topics about 10.0 Hz, `/map` 1.0 Hz, and wheel odometry about 20 Hz. Collision
Monitor activated without lifecycle errors and the gate correctly remained
closed when no fresh command existed.

Two traps cost real time here and will recur:

- Killing the `ros2 launch` parent leaves its children running. They keep
  their node names, so the next launch fails to activate Collision Monitor.
  Kill the process group, then confirm with `pgrep -fa`, and remember that
  orphans reparent to init and no longer share the launch's group id.
- `pgrep -f <pattern>` matches the invoking shell's own command line. Use
  `pgrep -fc "[c]ollision_monitor"` or every count is inflated by one.

### Deploying from a Windows workstation

The rover scripts start via `#!/usr/bin/env python3`. If the file reaches the
Jetson with CRLF line endings the kernel looks for an interpreter literally
named `python3\r` and the node dies at launch with:

```text
/usr/bin/env: 'python3\r': No such file or directory
```

The failure is easy to misread, because the rest of the launch comes up
normally and only one node is missing. The repository now carries a
`.gitattributes` pinning `*.py`, `*.sh`, `*.yaml` and `*.xml` to `eol=lf`. When
copying files outside git (`pscp`, `scp` from a Windows checkout), run
`sed -i 's/\r$//'` on the rover afterwards and confirm with
`head -1 <file> | od -c`.

### Hardware SLAM package added to this repository

This repository now contains `src/leo_rover_real_bringup`, a small package that
does not depend on Gazebo. It provides:

- `config/slam_params.yaml`: wall time, real frames/topics, 5 cm resolution,
  and 5 cm/radian scan insertion thresholds.
- `launch/slam.launch.py`: configurable static LIDAR transform plus asynchronous
  SLAM Toolbox. This is the LIDAR-only fallback.
- `launch/safe_mapping.launch.py`: calibrated depth filtering, base-frame
  LIDAR/depth fusion, optional wheel-only odometry TF, SLAM Toolbox, a
  fail-closed command gate, Nav2 Collision Monitor, and an optional bounded
  explorer.
- `scripts/depth_height_filter.py`: transforms aligned depth into
  `base_footprint` before applying ground/height filters.
- `scripts/scan_fusion.py`: transforms and self-filters the LIDAR, then emits
  separate collision and mapping scans.
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

### Rover 4 base-frame depth filtering and fusion (2026-08-13)

The RealSense is pitched down, so filtering image rows would mix floor and
obstacles. The implemented path first projects each aligned depth pixel using
the camera intrinsics, transforms the resulting 3D points into
`base_footprint`, and only then filters by base-frame Z height. Eight stationary
depth frames gave a floor-plane fit with 89.5% inliers, 4.44 mm p95 residual,
camera height 0.389 m, downward pitch 11.56 degrees, and roll about -0.25
degrees. The launch defaults round this to `camera_z:=0.393`,
`camera_pitch:=0.209`, and zero roll. Aligning the height-filtered camera scan
against the LIDAR gave `camera_yaw:=-0.035` rad (about -2 degrees): its residual
was 0.083 m versus 0.230 m at zero yaw.

The two camera products serve different purposes:

- `/camera/scan_collision`: points 0.04-0.45 m above the floor, 0.20-3.0 m
  range. This broad band retains low obstacles and furniture for safety.
- `/camera/scan_slam`: points 0.18-0.31 m above the floor, 0.20-5.0 m range.
  This narrow band approximates the LIDAR plane and avoids turning the floor or
  table tops into false 2D walls.

`scan_fusion.py` transforms the raw LIDAR into the base frame, applies only the
measured bounded rover self-mask, and publishes:

- `/scan_lidar_base`: corrected LIDAR only, useful as a diagnostic/baseline.
- `/scan_collision_fused`: nearest LIDAR or broad-band camera return, capped at
  3 m; used by both Collision Monitor and the explorer.
- `/scan_slam_fused`: nearest LIDAR or narrow-band camera return; used by SLAM
  Toolbox with a 12 m LIDAR ceiling.

The collision model uses a 0.35 m approach circle: the 0.44 x 0.44 m chassis
needs 0.311 m just to enclose its corners, with the remainder reserved for
mount/extrinsic uncertainty. Its directional time-to-collision action still
allows motion away from an obstacle.

If valid depth falls below 5%, the depth node deliberately publishes nothing.
The command gate requires a fresh raw camera-derived collision scan within 0.5
s, so a covered, disconnected, invalid, or transform-less camera closes motion
instead of allowing a LIDAR-only command. SLAM may continue LIDAR-only while
the camera is stale; this does not weaken the motion gate.

Rover 4's `robot_supervisor_rgb` keeps a `/cmd_vel` publisher endpoint even
when disabled. The gate conditionally tolerates that endpoint only while it
can freshly read the node's boolean `enabled` parameter as `false`. A missing
parameter service, stale check, `enabled:=true`, or any other unexpected final
publisher closes the gated command path. Keep the supervisor disabled for this
mapping stack.

Stationary live validation on Jetson-04 produced 83 finite collision-camera
bins and 114 mapping-camera bins in a representative frame. The fused mapping
scan had 416 finite bins versus 362 for corrected LIDAR alone. A matched
stationary SLAM probe produced 1,943 known cells with fusion versus 1,646 with
LIDAR alone (18.0% more known cells). All measurements were taken without
sending a velocity command.

Use `safe_mapping.launch.py` for this fused path. Keep the explorer disabled
until a physical operator is beside the rover and the stationary checklist is
green:

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
export ROS_DOMAIN_ID=<verified-domain>
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false publish_lidar_tf:=false \
  publish_odom_tf:=false publish_camera_tf:=true \
  start_sensor_fusion:=true start_slam:=true start_safety:=true
```

Do not run a second `slam_toolbox`, odometry TF publisher, or LIDAR TF
publisher alongside this command. On Rover 4, stop `leo-nav` first as described
above, while leaving the sensor/firmware services running. Reverse remains
disabled by default. The fusion is stationary-validated, but the first moving
run must still be a supervised, short, low-speed test before a room exploration
or final map save.

`mapping_artifact_recorder.py` runs by default with this launch. It publishes
the corrected map-frame route as `/exploration_path` and writes the following
to `~/leo_maps` on graceful shutdown:

- a standard `.pgm` plus `.yaml` occupancy map;
- `_path.csv` with timestamp, map X/Y, and yaw for every retained pose;
- `_path.png` with green start, red route, and blue final position;
- `_summary.json` with known cells, route length, and endpoints.

To checkpoint these files while SLAM is still running, call:

```bash
ros2 service call /save_mapping_artifacts std_srvs/srv/Trigger '{}'
```

Use `artifact_output_directory:=<directory>` and
`artifact_prefix:=<name>` to choose another destination/name. This recorder is
not a replacement for SLAM Toolbox pose-graph serialization when a resumable
mapping session is required.

`map_coverage_reporter.py` also runs by default. Every five seconds it reports
the mapped free area, unknown/free/occupied fractions, and the largest
reachable frontier clusters in map coordinates. A run should not be called
complete while meaningful reachable frontiers remain. `no reachable
frontiers` means the current occupancy grid is covered or the unknown space is
blocked; inspect the final map before deciding which. The reporter is
read-only and never publishes velocity—frontier-directed Nav2 driving remains
outside this conservative stack until its controller output is routed through
and motion-tested with the same gate and Collision Monitor.

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
- enforces an 11.50 V battery floor.

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
physical autonomous test; do not lower the safety package's 11.50 V floor to
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

Nav2 overlay bringup, fail-closed command chains, and the 2026-08-20
“topics never start” failure mode are in
[`deploy/jetson04/MOTION_STACK.md`](deploy/jetson04/MOTION_STACK.md).

### Rover 4 readiness session, no firmware dropout under full stack (2026-08-14)

State found on connection (fresh boot, battery 12.00 V):

- `leo_real` no longer exists anywhere on the machine. `~/ros_ws/install` was
  rebuilt down to only `leo_nav_bridge`, so `leo-ros.service` (which launched
  `leo_real leo_real.launch.py`) was crash-looping every 10 s with "Package
  'leo_real' not found" (restart counter 33+), and `leo-nav.service` was dead
  for the same reason. `leo-ros` was stopped for the session; both units need a
  maintenance decision (rebuild `leo_real` or rewrite the units).
- With `leo-ros` gone, nothing starts the RealSense. The stock driver works as
  a replacement and matches the topics `safe_mapping.launch.py` expects:
  `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
  rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15`
  (installed version 4.58.3; older `rgb_camera.profile` names are not
  declared). Aligned depth then runs at 15 Hz, and the depth filter reported
  87.5% depth health.
- The active workspace for `leo_rover_real_bringup` is `~/leo_sensor_ws`
  (symlink-install, byte-identical to this repository on 2026-08-14).
  `~/ros2_ws/src/leo_rover_real_bringup` is a stale variant missing
  `scan_fusion.py` and `depth_height_filter.py` even though its install dir
  was rebuilt on 2026-08-14 — do not launch from it. `~/codex_ws` is older
  still.
- Since `leo-nav` no longer starts, there is nothing to stop before taking
  `/cmd_vel`; the daemon restart plus `safe_mapping.launch.py` sequence below
  otherwise still applies.

Verified running configuration (all stationary, no motion commanded):
boot services `lidar`, `lidar-tf` (corrected `y=0.04` override active,
`[0.077, 0.040, 0.246]` confirmed live), `leo-nav-bridge`, `rosbridge`, plus
the RealSense launch above, plus `safe_mapping.launch.py start_explorer:=false
publish_lidar_tf:=false publish_odom_tf:=false publish_camera_tf:=true
start_slam:=true start_safety:=true` from `~/leo_sensor_ws`. Rates: `/scan`
10.0 Hz, camera scans ~13 Hz, fused scans ~10 Hz, `/map` 1.0 Hz, firmware
wheel odom 20.0 Hz, firmware IMU 100.6 Hz, battery telemetry 10.0 Hz.
Collision Monitor activated cleanly; command chain had a single owner at every
hop and zero foreign `/cmd_vel` publishers; the gate heartbeated zeros and
Collision Monitor stayed silent on zero input as documented.

Firmware starvation did NOT recur: a 5-minute monitor under the full stack
showed battery telemetry at exactly 10.0 Hz in every 30 s bin and
`enP8p1s0` traffic steady at ~73 KB/s to the rover (~5% of the 1396 KB/s
failure level; idle baseline that day was 24 KB/s). Rosbridge client count was
0 — keep the rover web UI closed. Total stack uptime exceeded 12 minutes with
zero dropouts, versus death in ~60 s during the 2026-08-13 incident. The
lighter camera profile (640x480x15 instead of 1280x720x30) is part of keeping
local load down; keep it.

Monitoring trap: firmware topics are best-effort, and
`/rob_2/firmware/wheel_odom` is `leo_msgs/WheelOdom`, not `nav_msgs/Odometry`.
A subscriber with default QoS or the wrong type silently counts zero and looks
exactly like a dropout. `ros2 topic hz` is type-agnostic and the safer probe.

A stationary map checkpoint (176x107 cells, 2,374 known) was saved via
`/save_mapping_artifacts` to `~/leo_maps/leo_room_20260814_122401.*`; local
copies are under `artifacts/jetson04_readiness_20260814/`. Key-based SSH for
the operator workstation was installed in `~/.ssh/authorized_keys`.

### Rover 4 first autonomous room exploration (2026-08-14)

A supervised 180 s bounded exploration ran the same day on the stack described
above. Result: 7.29 m traveled, 75.2% of the room mapped (15,842 known cells,
180x117 @ 5 cm), obstacle avoidance exercised live, zero firmware dropouts,
zero collisions. Artifacts, labeled debug video, and the full rosbag are in
`artifacts/jetson04_exploration_20260814/`; rover-side copies in `~/leo_maps`
and `~/leo_bags/explore_20260814_run3`. The SLAM pose graph was serialized to
`~/leo_maps/leo_room_explored_20260814_posegraph.*`, so the session is
resumable.

Three false starts preceded the successful run; each is a production lesson:

1. **Standalone explorer defaults do not match the fused stack.** `ros2 run
   ... safe_room_explorer.py` with only `run_duration`/`max_distance` (the
   guide's old example) dies with "stale depth-camera scan": its defaults are
   `camera_scan_topic=/camera/scan`, `scan_topic=/scan`,
   `odom_topic=/wheel_odom_integrated`, `battery_topic=/firmware/...` — all
   wrong here. Pass the same parameters `safe_mapping.launch.py` gives it
   (`scan_topic:=/scan_collision_fused`, `scan_yaw_offset:=0.0`,
   `camera_scan_topic:=/camera/scan_collision`, `camera_scan_yaw_offset:=0.0`,
   `odom_topic:=/wheel_odom`,
   `battery_topic:=/rob_2/firmware/battery_averaged`), or use
   `start_explorer:=true` on the launch instead.
2. **rosbag on `/cmd_vel_raw` closes the gate.** The gate audits subscribers
   of its raw output and fails closed on unknown consumers
   (`unexpected raw command consumers: rosbag2_recorder`); a `ros2 topic echo`
   does the same. Either record everything except `/cmd_vel_raw` (gate
   decisions are still recoverable from `/cmd_vel_request`, `/cmd_vel`, and
   `/rosout`), or extend the gate's `allowed_cmd_vel_raw_subscribers`
   parameter at launch time.
3. **Bagging raw images starves the control nodes.** Recording 640x480 raw
   color+depth pushed load average to 9.3 on 6 cores and produced 0.4-0.5 s
   arrival gaps on the fused scan — tripping the explorer's 0.4 s
   `scan_timeout`. Bag a throttled color stream (a 5 Hz relay ~4 MB/s) and
   skip raw depth: the camera's obstacle decisions are already in
   `/camera/scan_collision`. Explorer margins `scan_timeout:=0.8`,
   `camera_scan_timeout:=0.8`, `output_timeout:=1.5` are safe because the
   gate's stricter 0.4/0.5 s arrival checks still govern actual motion.

Findings from the successful run, established from the bag (ground truth):

- **The run's end was a safety stop, not a fault.** At t≈130 s an obstacle sat
  at 0.35 m on the left flank — exactly on Collision Monitor's 0.35 m approach
  circle — and CM held output near zero for 40 s. The explorer's own sector
  view said front was clear (1.27 m > its 0.55 m stop distance), so it kept
  requesting forward instead of triggering its CM-held-turn recovery. That
  hold-detection gap is the top explorer bug to fix for production: it
  compares output freshness, not output magnitude vs request.
- **Firmware never faltered under the running stack plus motion**: wheel odom
  20 Hz, IMU 100 Hz, battery 10 Hz continuous across the whole bag; link
  steady at ~85-90 KB/s. The 2026-08-13 starvation did not recur.
- **Per-endpoint DDS subscription failures are real and misleading.** The
  explorer's `/wheel_odom` subscription stopped receiving for 13 s (its abort
  reason "stale wheel odometry") while the bag and the gate on the same topic
  received every message; after the run, fresh `ros2 topic hz` probes of
  `/rob_2/firmware/wheel_odom` reported nothing while established
  subscriptions flowed at 20 Hz. A "dead" topic probed by a single new
  subscriber is not proof the publisher is down — cross-check with an
  already-attached consumer (the bag, the bridge) before blaming firmware.
- Battery sagged 12.00 → ~11.5 V averaged over the ~75 min session; recharge
  before the next motion session.

### Rover 4 exploration runbook (validated 2026-08-14)

The complete procedure for an autonomous mapping run with debug recording.
Every step was executed and validated live; deviations that looked harmless
cost real time (see the false-start list in the previous section).

1. **Preflight** (see "First connection checklist"): battery comfortably above
   11 V, `lidar`/`lidar-tf`/`leo-nav-bridge` services running, `leo-nav` NOT
   running, exactly one `/cmd_vel` publisher after the stack is up.
2. **Camera** (leo_real is gone from the machine):
   `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
   rgb_camera.color_profile:=640x480x15 depth_module.depth_profile:=640x480x15`
3. **Stack** from `~/leo_sensor_ws` (managed process, own PGID):
   `ros2 launch leo_rover_real_bringup safe_mapping.launch.py
   start_explorer:=false publish_lidar_tf:=false publish_odom_tf:=false
   publish_camera_tf:=true start_sensor_fusion:=true start_slam:=true
   start_safety:=true`
   Then verify: Collision Monitor activated in the log, AND
   `/collision_monitor/footprint` has 1 publisher + 1 subscription — the
   approach polygon is fed by `footprint_publisher.py`; if that node is
   missing the polygon is EMPTY and collision avoidance silently checks
   nothing (Humble approach polygons have no static points parameter).
4. **Debug recording**: start `tools/debug_color_throttle.py`, then the
   standard bag topic set from `tools/README.md`. Never bag `/cmd_vel_raw`
   (gate closes) or raw images (CPU starvation). Optionally start
   `tools/firmware_stability_monitor.py`.
5. **Explorer** — always pass the full wiring (standalone defaults are wrong
   for this stack):
   `ros2 run leo_rover_real_bringup safe_room_explorer.py --ros-args
   -p run_duration:=120.0 -p max_distance:=12.0
   -p linear_speed:=0.092 -p angular_speed:=0.276
   -p front_stop_distance:=0.45 -p front_clear_distance:=0.60
   -p minimum_turn_clearance:=0.40
   -p scan_topic:=/scan_collision_fused -p scan_yaw_offset:=0.0
   -p camera_scan_topic:=/camera/scan_collision -p camera_scan_yaw_offset:=0.0
   -p odom_topic:=/wheel_odom
   -p battery_topic:=/rob_2/firmware/battery_averaged
   -p scan_timeout:=1.5 -p camera_scan_timeout:=1.5 -p output_timeout:=2.0`
   The relaxed explorer watchdogs are safe: the gate still enforces 0.4/0.5 s
   arrival freshness on every command it forwards. Re-running the explorer
   continues the SAME map while the stack stays up — multi-leg runs are the
   normal way to reach full coverage.
6. **Known stop conditions**: driving nose-first into a pocket blinds the
   RealSense (<5% valid depth → depth node publishes nothing → gate closes →
   next explorer start refuses with "stale depth-camera scan"). Reposition
   the robot to face open space (operator) and relaunch; depth health
   recovers immediately. Watch `depth_height_filter` log lines.
7. **Artifacts** (while SLAM is still alive): checkpoint any time with
   `ros2 service call /save_mapping_artifacts std_srvs/srv/Trigger '{}'` →
   `~/leo_maps/`; final map with `nav2_map_server map_saver_cli`; resumable
   pose graph with
   `ros2 service call /slam_toolbox/serialize_map
   slam_toolbox/srv/SerializePoseGraph '{filename: <path>}'`.
   Render the labeled analysis video from the bag with
   `tools/render_labeled_debug_video.py`.

Tuning state as of 2026-08-14 (after the table-leg deadlock): CM approach
footprint is the chassis rectangle `[0.28/-0.26 x, +/-0.26 y]` published on
`/collision_monitor/footprint`, `time_before_collision` 1.2 s; explorer speeds
0.092/0.276 (+15%); explorer hold-detection requires 0.5 s of sustained
nonzero CM output before clearing (a flickering CM previously reset the
detector and the robot sat 40 s against a table leg).

### Rover 4 doorway-transit findings (2026-08-14 evening)

Two more full 5-minute runs completed (26.5 m and 23.8 m — both ended on the
wall clock; boxed-probe freed a 0.32 m pocket in 21.8 s unassisted). Passage
mode reliably NAVIGATES TO doorways but transit was not yet achieved; the
final attempts ran out of battery (10.2 V floor). Fixes committed for the
next session, all deployed to `~/leo_sensor_ws` (symlinked):

- Gap aim = cartesian midpoint of the door posts (angular midpoint drives
  into the near post when one edge is close).
- CM silence while requesting motion is a HOLD (escape fires), not a data
  stall: the old separate liveness-pause reset the hold timer every 2 s and
  pinned the robot at the door indefinitely.
- Passage footprint front margin trimmed to 2 cm; a post 3 cm from the old
  edge flickered inside the polygon and froze all motion both directions.
- Boxed-probe no-progress exit requires side clearance, not just a clear
  front wedge.

Operational notes: the coverage watcher must not be armed right after a
stack restart (fresh SLAM + stationary robot reads "no reachable frontiers"
and kills the run at min-runtime); the dynamic slim footprint is verified
live (0.52 -> 0.48 m on `/passage_active`); speed ceilings are now 0.18
(explorer and gate) with 0.163 m/s cruise validated. The 2026-08-14 full-run
map artifacts and labeled videos are in
`artifacts/jetson04_exploration_20260814/`.

### Rover 4 FIRST NARROW-PASSAGE TRANSIT and session wrap (2026-08-14 late)

**The rover autonomously drove through the narrow doorway and mapped the
second room** (operator-confirmed; the two-room map with the transit path is
`artifacts/jetson04_exploration_20260814/leo_room_20260814_154151_path.png`,
rosbag `explore_20260814_door5`). What made transit work, in order of
importance:

1. Camera obstacle band floor raised 0.04 -> 0.10 m
   (`depth_height_filter collision_min_height`): the 4 cm floor turned door
   sills and white-panel stereo noise into ghost walls across clear
   passages. This was the dominant blocker.
2. Dynamic slim CM footprint (0.48 m wide, 2 cm front margin) published on
   `/passage_active` during passage mode; static 0.52 m footprint left only
   ~4 cm/side in the door and noise flicker-vetoed everything.
3. Cartesian gap aim (midpoint of the door posts, not the angular window
   centre) plus align-in-place when the gap is >23 deg off-axis; oblique
   entries put a post on the flank where no rotation is legal.
4. Passage hysteresis (3 s) and passage hold patience (4 s vs 1.2) so
   flickering detections do not abort an approach.

Remaining known weakness for the next session: **flank wedges**. The gap
detector can qualify a dead-end pocket between panels as a passage
(mitigation deployed: `gap_range` free-depth raised to 2.0 m at run time),
the robot enters, and an obstacle inside the corner-sweep circle (~0.35 m)
makes every pure rotation illegal; one wedge took 165.7 s of boxed-probe
cycling to escape and another consumed a whole run. Arc escape candidates
(forward/reverse combined with turn) are committed and deployed but NOT yet
field-tested — they should cut wedge escapes to seconds. Side-corridor
avoidance (veer away when a flank closes under 0.40 m) is active in plain
forward motion but intentionally not during passage approach.

Watchdog architecture lesson that cost hours: never give two watchdogs
overlapping authority. The CM-output-liveness pause reset the hold-escape
timer every 2 s and the robot sat pinned at the door indefinitely; CM
silence while requesting motion is a HOLD (hold detector's job), not a data
stall. Similarly, the coverage watcher must not be armed right after a SLAM
restart with a stationary robot.

End-of-session state: mapping stack and camera STOPPED on the Jetson (boot
services lidar/lidar-tf/leo-nav-bridge/rosbridge left running as found);
battery 11.96 V; all four modified scripts byte-identical between the local
repo and `~/leo_sensor_ws`; all maps, pose graphs, logs, labeled videos and
the transit bag copied to `artifacts/jetson04_exploration_20260814/`. The
day's full history is in this guide's dated 2026-08-14 sections; the
exploration runbook above remains the canonical procedure.

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

For LIDAR plus RealSense operation, use the repository's
`safe_mapping.launch.py` path described above. It performs the transform and
base-height filtering before fusing the data and keeps a single Collision
Monitor output. Raw depth topics alone are not evidence that fusion is active;
verify the two camera scans and two fused scans are fresh.

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
