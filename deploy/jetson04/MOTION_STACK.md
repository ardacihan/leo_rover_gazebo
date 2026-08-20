# Rover 4 motion stack — why it looks down, and how to start it

Written 2026-08-20 after watching a live Nav2 overlay session on
`jetson-04` (`192.168.178.104`, `ROS_DOMAIN_ID=4`). Use this when a
session reports “topics never start” or the rover will not move even
though `/scan`, `/wheel_odom`, and `/map` appear on the graph.

The field guide with the validated 2026-08-14 mapping run is
[`REAL_ROVER_GUIDE.md`](../../REAL_ROVER_GUIDE.md). Host TF/lidar notes
are in [`README.md`](README.md).

## The actual failure

Motion is not failing because topics never come up. Sensors and the
overlay usually start. The command chain is **fail-closed**: any hop
that looks stale publishes **zero**. The robot sits still while the
graph still lists `/scan`, `/wheel_odom`, and `/map`.

Treat a fresh `ros2 topic list` as a hint, not proof. A process list
plus one already-attached consumer (or `leo_nav2_preflight`) is more
trustworthy than a new `ros2 topic hz` process.

## Two stacks, two command paths

Do not mix them. Do not launch both.

### Validated mapping path (moved on 2026-08-14)

Workspace: `~/leo_sensor_ws`.

```text
bounded explorer       -> /cmd_vel_request
safety_command_gate    -> /cmd_vel_raw
collision_monitor      -> /cmd_vel
Leo firmware           <- /cmd_vel
```

Launch:

```bash
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false publish_lidar_tf:=false \
  publish_odom_tf:=false publish_camera_tf:=true \
  start_sensor_fusion:=true start_slam:=true start_safety:=true
```

Use this path when the goal is “move and map”, not “exercise Nav2”.

### New Nav2 overlay (heavier, still being proven)

Workspace: `~/leo_nav2_ws`. Package: `leo_nav2_exploration`.

Designed chain:

```text
Nav2 controller        -> /cmd_vel_nav
velocity_smoother      -> /cmd_vel_smoothed
velocity_guard         -> /cmd_vel_guarded
collision_monitor      -> /cmd_vel
Leo firmware           <- /cmd_vel
```

Launch:

```bash
ros2 launch leo_nav2_exploration real_navigation.launch.py
```

Frontier exploration is a **second** launch, cold-idle by default:

```bash
ros2 launch leo_nav2_exploration frontier_exploration.launch.py \
  profile:=real_root autostart:=false
```

Do not send a `NavigateToPose` goal or start the explorer until
Collision Monitor is active, `/cmd_vel` has a single owner, and
preflight is green.

**2026-08-20 workaround in tree:**
`config/real/collision_monitor.yaml` currently sets
`cmd_vel_in_topic: /cmd_vel_smoothed`, bypassing `velocity_guard`.
That was a live load-shedding edit because the Python guard’s scan
subscription starved and failed closed. Collision Monitor and the
controller still gate motion. Restore `/cmd_vel_guarded` only after
the guard can keep up on this Jetson.

## Why “missing topics” is usually a lie on this robot

These traps are documented from rover 4 itself. They mimic a stack
that never started.

### CPU and DDS starvation

The Jetson has six cores. Nav2 (planner, controller, BT, costmaps) +
SLAM + scan filter + camera pointcloud into the local costmap + a
Python `velocity_guard` at 20 Hz is enough to push load over the
machine. Then:

- firmware odometry arrives in bursts over the serial/Ethernet path
- the guard’s LaserScan callback falls behind
- the guard reports `scan_stale` / `odom_stale`
- `/scan` can still be ~10 Hz for everyone else

`velocity_guard.yaml` already widens timeouts for this
(`scan_timeout: 1.5`, `odom_timeout: 2.5`). Widening them further
does not fix a starved callback.

### New subscribers lie

A fresh `ros2 topic hz` or `echo` can see nothing while an
already-attached node on the same topic is fine. Per-endpoint DDS
failures were measured on 2026-08-14: the explorer’s `/wheel_odom`
reader died for 13 s while the bag and the gate received every
message. Cross-check with a consumer that is already up before
blaming firmware or restarting the stack.

### Multicast to the rover SBC

The rover’s onboard computer shares `ROS_DOMAIN_ID=4` over
`enP8p1s0`. Heavy Jetson traffic (camera, depth, maps, costmaps) can
saturate that link. Firmware topics freeze. It looks like dead
hardware. It recovered in 2026-08-13 after the stack went quiet, with
no power cycle.

Idle to-rover traffic is tens of KB/s. Failure was ~1.4 MB/s. Close
the rover web UI (`10.0.0.1:9090`) before changing anything else.
Keep camera at **640×480×15**. Do not bag raw images while debugging
motion.

### Two workspaces and boot services

| Location | Role |
|---|---|
| systemd `lidar`, `lidar-tf`, `leo-nav-bridge` | keep running |
| systemd `leo-nav` | keep **off** while this overlay owns `/cmd_vel` |
| `~/leo_sensor_ws` | validated `safe_mapping` stack |
| `~/leo_nav2_ws` | new Nav2 overlay |
| `~/ros2_ws`, `~/codex_ws` | stale; do not launch from them |

`pkill` of `ros2` / `slam` / `nav2` leaves camera and launch children.
The next start then fights USB (RealSense V4L2) and duplicate TF
parents (`laser_frame`, `camera_link`).

`.bashrc` on this host has accumulated conflicting `ROS_DOMAIN_ID`
exports. Noninteractive SSH must export `ROS_DOMAIN_ID=4` explicitly.
The ROS CLI daemon caches stale node/topic names.

## What not to do in an agent session

These moves were the live 2026-08-20 session. Each one makes a healthy
graph look dead.

1. **Broad `pkill`.** The guide forbids it on a shared robot. Kill only
   the process group you started (`kill -INT -- -$pgid`).
2. **Goals before the graph is quiet.** Starting the explorer or a
   Nav2 goal while the guard already reports `scan_stale` just produces
   controller aborts and more load.
3. **Probe storms.** Each `ros2 topic echo/hz/info` is another DDS
   participant. That is how a 10 Hz scan becomes a “stale” scan.
4. **Live surgery.** Editing Collision Monitor, enabling RealSense
   decimation, and killing `stuck_recovery` mid-run changes the graph
   under the nodes you are trying to measure.
5. **Bagging `/cmd_vel_raw` or raw images.** The old gate fails closed
   on unknown `/cmd_vel_raw` readers. Raw 640×480 color+depth pushed
   load average to 9.3 and tripped scan timeouts.
6. **Mixing stack wiring.** Standalone `safe_room_explorer.py`
   defaults (`/camera/scan`, `/wheel_odom_integrated`,
   `/firmware/...`) do not match either current rover-4 launch.

## Bringup that can actually produce motion

One stack. Wait. Then one command. A person must be next to the robot.

1. Preflight: battery comfortably above 11 V, `who` is expected,
   `lidar` / `lidar-tf` / `leo-nav-bridge` up, `leo-nav` down, rover
   web UI closed.
2. Camera, if not already up:

   ```bash
   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
     rgb_camera.color_profile:=640x480x15 \
     depth_module.depth_profile:=640x480x15
   ```

3. **One** overlay from the matching workspace, in its own process
   group (see “Managed process pattern” in the field guide). Do not
   also start `safe_mapping.launch.py` if you started
   `real_navigation.launch.py`.
4. One preflight, not a probe loop:

   ```bash
   ros2 run leo_nav2_exploration preflight_check --ros-args \
     -p --profile real_root
   ```

   Wait until Collision Monitor is active and `/cmd_vel` has exactly
   one publisher.
5. Then one short `NavigateToPose` **or** one bounded explorer, not
   both, not immediately after a restart.
6. On failure, read `/rosout` and the guard/CM reason from nodes that
   are already subscribed. Do not spawn ten new CLI processes. Do not
   `pkill`.

Load-shedding that is actually allowed if the Jetson is hot, in order:

- keep the rover UI closed
- keep camera at 640×480×15
- leave the explorer off until Nav2 follows one pose
- disable the camera costmap source (`enable_voxel:=false`) before
  bypassing `velocity_guard`
- never bag raw depth while commanding motion

## Quick diagnostics (low load)

Watch to-rover traffic instead of guessing firmware death:

```bash
read -r r1 t1 < <(awk '/enP8p1s0/{gsub(/:/," ");print $2, $10}' /proc/net/dev)
sleep 10
read -r r2 t2 < <(awk '/enP8p1s0/{gsub(/:/," ");print $2, $10}' /proc/net/dev)
echo "to rover $(( (t2-t1)/10/1024 )) KB/s"
```

Anything far above ~50–90 KB/s is the 2026-08-13 starvation pattern
returning. A ping to `10.0.0.1` climbing into double-digit milliseconds
says the same thing.

Confirm command ownership without a new velocity publisher:

```bash
timeout 5s ros2 topic info -v /cmd_vel
ps -eo user:12,pid,ppid,pgid,lstart,stat,args | \
  grep -E '[r]os2|[s]lam|[n]av2|[t]eleop|[r]ealsense'
```

If Collision Monitor is silent, Humble can stay quiet while its input
is zero. That is not proof the node is down.
