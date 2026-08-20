# Leo Rover — mapping an office, on the real robot

Written overnight 2026-08-19/20 for a first hardware run. Everything here was validated in
simulation against a skid-steer odometry error model; the parts that were *not*
validated on hardware are marked **UNVERIFIED ON HARDWARE** rather than left
for you to discover.

Read [§1](#1-the-short-version) and [§2](#2-before-you-power-anything-on), do
[§3](#3-bring-up-order), and keep [§7](#7-when-something-goes-wrong) open.

---

## 1. The short version

Two stacks ship. **Start with the first one.**

| | `real_mapping.launch.py` | `real_navigation.launch.py` (+ `real_exploration.launch.py`) |
|---|---|---|
| what drives | you, with a joystick | Nav2 + frontier explorer |
| what it runs | scan filter, SLAM, safety chain | that, plus planner, controller, BT, explorer |
| ways to fail | few | **in simulation, five autonomous runs in fourteen stopped early and never restarted** |
| use it | **first run, and any run where the map matters** | only after the first one worked |

The map does not care who steers. Autonomous exploration is the part that is
fragile; SLAM is not. Drive it yourself the first time.

Across fourteen simulated runs of the shipped configuration, nine mapped the
whole building (96.7–98.0% coverage) and **five stopped early** (30%, 52%, 57%,
65%, 85%). The failure is always the same and always benign: the rover drives
for a few minutes, stops, and never restarts while Nav2 keeps sending it goals.
**Zero contacts in all fourteen**, one near-miss — you get a partial map, not a
damaged robot.

That is close to a one-in-three chance of an unattended autonomous run not
finishing. It is not a reason to distrust the map it *did* build, and it is a
very good reason to drive the first one yourself.

**One cause of that was found and fixed in the last hour before this was
written**, so the shipped numbers above are pessimistic. Nav2's `BackUp`
recovery had 10 s to travel 0.25 m at 0.04 m/s; next to an obstacle the
collision monitor cuts commands to 75%, which needs 8.3 s — so the backup kept
timing out just short of finishing, and each timeout permanently blacklists the
frontier the rover was heading for. With a 20 s allowance and 0.08 m/s
(`scripts/apply_recovery_timeout.py`, already applied), **all three of the
worst-stalling seeds recovered: 52%, 30% and 65% coverage became 98%, 98% and
98%**, with the timeout log line gone in every one. Believe it enough to expect
far fewer stalls — not enough to leave the rover unattended.

If you do run it autonomously, **keep a controller in reach**. If the rover has
not moved for ~30 seconds:

```bash
# 1. Ctrl-C the real_exploration.launch.py terminal. This cancels its goal;
#    teleop and an active Nav2 goal both write /cmd_vel_nav and will fight.
# 2. Drive it a metre or two clear by hand:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_nav
# 3. Restart the explorer.
ros2 launch leo_nav2_exploration real_exploration.launch.py
```

Do not wait it out. Once the explorer has blacklisted every frontier it stops
for good, and blacklisting is not undone by the rover starting to move again.

## 2. Before you power anything on

**Print the ArUco markers.** `scripts/make_aruco_models.py` writes
`src/leo_rover_gazebo/models/aruco_markers/textures/aruco_<id>.png` (dictionary
`DICT_4X4_50`, ids 1–8). Print at least 15 cm across, on matte paper — gloss
blows out under office LEDs and the detector loses the corners.

**Then measure the printed black square with a ruler and use that number.**
Not the sheet, not the white border, not what the printer dialog said. This is
the one parameter that fails silently: get it wrong by 30% and every marker is
reported 30% too near or too far along the camera's view ray, with no error
anywhere. Pass it as `marker_length:=<metres>`.

Tape them flat at roughly camera height (~30 cm) with the white border intact.

How accurate to expect: with the map frame taken out of the equation, the
detector placed 7 of 8 simulated markers with a **median error of 0.26 m at
3–5 m range and zero false positives**, and the implied marker length came back
within 5% of the configured value. During real mapping the marker positions
also carry the map's own drift, so treat them as "which room, roughly where",
not as survey points.

**Tell the detector which ids you placed.** `allowed_ids` defaults to
`1,2,3,4,5,6,7,8`; anything else is rejected before it can become a landmark.
This is not paranoia — a simulated run produced a spurious id 25 with no such
marker anywhere in the world. `DICT_4X4_50` has weak error correction and an
office is full of high-contrast rectangles. The `min_hits` gate suppressed that
one, but an allowlist does not depend on a false detection failing to repeat.
If you need many markers, `dictionary:=DICT_5X5_250` is markedly harder to
false-trigger — just print from the matching dictionary.

**Check what the rover publishes**, before launching anything of ours:

```bash
ros2 topic hz /scan            # expect ~10 Hz
ros2 topic hz /wheel_odom      # expect >= 20 Hz
ros2 topic echo --once /firmware/imu     # leo_msgs/Imu, if you want the EKF
ros2 topic hz /camera/camera/color/image_raw           # for ArUco
ros2 topic hz /camera/camera/depth/color/points        # for camera obstacles
ros2 run tf2_ros tf2_echo base_footprint laser         # lidar mount
```

If `ros2 topic hz` prints nothing for `/scan` or the camera topics, check
whether the topic actually is silent before believing it: those publishers use
best-effort sensor QoS and `ros2 topic hz` subscribes reliably by default, so
an incompatible-QoS subscription looks exactly like a dead sensor. Add
`--qos-reliability best_effort`, or use `ros2 topic echo --once`.

If the RealSense point cloud is genuinely missing, it is almost always that the
driver was launched without it — `pointcloud.enable:=true` (or `enable_pointcloud`,
depending on the wrapper version). The colour image and the point cloud are
separate switches.

**Measure and set the lidar and camera mounts.** The static transforms in
`leo_rover_real_bringup/launch/safe_mapping.launch.py` default to lidar
`(0, 0, 0.15)` and camera `(0.065, -0.020, 0.31)`. A 5 cm error in the lidar's
x offset puts a 5 cm bias into every scan match, and SLAM will happily build a
consistent, wrong map.

## 3. Bring-up order

Order matters: SLAM must start against an `odom` frame that already exists and
is not about to change owner.

```bash
# 1. rover firmware + lidar + camera (your existing bringup)

# 2. odometry — pick ONE of 2a or 2b

# 2a. wheel odometry only (what the rover already does)
#     nothing extra to launch

# 2b. wheel + gyro fusion (recommended; see §4)
ros2 launch leo_nav2_exploration odometry_fusion.launch.py

# 3. mapping + safety.
#    use_ekf:=false because step 2b already started the EKF; set it true
#    instead if you skipped 2b and want real_mapping to launch it itself.
#    marker_length is YOUR measured black square; allowed_ids are the ids
#    you actually taped up.
ros2 launch leo_nav2_exploration real_mapping.launch.py \
    use_ekf:=false \
    use_aruco:=true \
    marker_length:=0.15 \
    allowed_ids:=1,2,3,4

# 4. preflight — read every line
ros2 run leo_nav2_exploration preflight_check --profile real_root \
    --require-camera --require-imu --require-aruco --expect-ekf

# 5. teleop, into the TOP of the safety chain
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_nav
```

**Never publish to `/cmd_vel` directly.** The collision monitor owns that topic
and is the only thing standing between a joystick and a wall. Commands enter at
`/cmd_vel_nav` and pass through smoother → guard → monitor.

Watch in RViz: `/map`, `/scan_filtered`, TF, and `/aruco_markers`.

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/office_map
```

### If you want the rover to drive itself

Only after a teleop map has worked, and in this order:

```bash
ros2 launch leo_nav2_exploration real_navigation.launch.py   # SLAM + Nav2 + safety
# send two or three goals from RViz and watch it get there
ros2 launch leo_nav2_exploration real_exploration.launch.py  # then let it choose
```

The explorer is a separate launch on purpose: the rover should be navigating
correctly under *your* goals before anything starts picking its own.

The planner is **NavFn**, not `SmacPlannerLattice`. On the same world, same
seed, same everything else, the lattice planner missed its 5 Hz rate 48 times
against NavFn's 2, and the resulting map was squeezed along one axis with
partitions displaced by up to a metre — a starved planner steals CPU from the
scan matcher, and that is what a scale error in a map looks like. A state
lattice buys kinematic feasibility that a rover with a zero turning radius does
not need. Expect the gap to be wider on the rover's computer, not narrower.

## 4. The EKF, and the one thing that will break it

Fusing the gyro's yaw rate with the wheels' forward velocity halved odometry
error in simulation (7.5 m → 2.4 m of drift over a long run) and the
improvement propagated into the map. On a skid-steer chassis, wheel-derived
yaw is the worst-measured quantity on the robot: the wheels slide sideways
through every turn. A MEMS gyro is wrong by bias drift alone.

**Exactly one node may publish `odom -> base_footprint`.** The rover's existing
bringup does it from `wheel_odom_tf.py`. `odometry_fusion.launch.py` does it
from the EKF. Run both and tf2 will not complain — it accepts both transforms
and consumers see the pose flip between two estimates, which looks exactly like
a SLAM failure and is not.

So: **stop `wheel_odom_tf` (or launch it with `publish_tf:=false`) before
starting the EKF.** Then confirm:

```bash
ros2 topic info /tf --verbose | grep "Node name"   # should not list wheel_odom_tf
ros2 topic hz /odometry/filtered                   # ~30 Hz
```

`preflight_check --expect-ekf` checks both of these.

The bridge that makes this possible is `leo_imu_bridge`: the firmware publishes
`leo_msgs/Imu`, which `robot_localization` cannot read, so it is republished as
`sensor_msgs/Imu` with the orientation marked absent (the sensor has no
magnetometer and reports no attitude). **Keep the rover still for the first ~10
seconds** — the bridge estimates gyro bias from the first 200 samples. If it
was moving, restart it. 0.5 deg/s of uncorrected bias is 30 degrees of heading
after a minute, which is worse than the wheel odometry it replaces.

**UNVERIFIED ON HARDWARE**: the firmware IMU topic name (`/firmware/imu`, which
may be namespaced — the battery topic in the existing bringup is
`/rob_2/firmware/battery_averaged`), the gyro noise level, and whether the
bias-calibration window is long enough. Check the logged bias line before
trusting the fusion.

## 5. What each sensor is doing

| sensor | used for | how |
|---|---|---|
| RPLIDAR C1 | the map; obstacle layer; collision monitor | `/scan` → box filter → `/scan_filtered` → slam_toolbox + costmaps + collision monitor |
| wheel encoders | forward velocity | EKF input, and `odom_topic` for the controller |
| IMU (gyro z) | heading rate | EKF input |
| RealSense depth | obstacles the lidar plane misses | `PointCloud2` source on the costmap `ObstacleLayer`, 0.06–0.60 m height band |
| RealSense colour | ArUco markers | `aruco_detector` |

The depth camera is an **ObstacleLayer** source, not a VoxelLayer. The bundle's
VoxelLayer reported a frozen sensor origin, which disables raytracing, so
obstacles were marked and never cleared. A single false depth return would
have stayed in the costmap for the rest of the run.

The scan filter's box (`base_footprint`, ±0.22 m, z −0.10…0.80) exists because
the camera bracket puts a fixed return into the lidar between +45° and +82° at
0.06–0.17 m. SLAM must read `/scan_filtered`, never `/scan` — `config/real/slam.yaml`
already does.

## 6. SLAM settings, and why

`config/real/slam.yaml`, the values that were tuned rather than defaulted:

```yaml
max_laser_range: 12.0                  # the C1's actual reach; 8.0 threw away returns
scan_buffer_maximum_scan_distance: 12.0
loop_search_maximum_distance: 8.0      # was 3.0 -- drift exceeds 3 m before
                                       # loop closure ever gets a chance
loop_search_space_dimension: 10.0      # was 6.0
loop_match_minimum_chain_size: 5       # NOT lowered
loop_match_minimum_response_coarse: 0.45  # NOT lowered
scan_topic: /scan_filtered
```

The loop-closure pair is the important part, and the asymmetry is deliberate:
**widening where loop closure looks is safe; lowering what it accepts is not.**
Relaxing the acceptance thresholds improved every aggregate metric in
simulation *and* introduced a false closure that drew one wall twice, 0.30 m
apart, with mapped free space in between — a gap a planner would have driven
through into solid wall.

**Long corridors are the known weak spot.** A 24 m corridor exceeds the 12 m
lidar, so the scan matcher has nothing to lock onto longitudinally and error
grows along the corridor axis. In rooms and partitioned spaces the same
configuration is reliably excellent. If your office has a long featureless
hallway: **drive a loop** — down one side and back — rather than out and back
along the same line. Closing a loop is what lets the optimiser fix it.

### What a good map looks like

From simulation, so treat these as the shape of the answer rather than targets:

| | rooms and partitions (`depot_world`) | five rooms + a 24 m corridor (`office_world`) |
| --- | --- | --- |
| wall error | 4 cm | 4–29 cm (mean 14 cm) |
| trajectory error | 7–12 cm | 44–68 cm (mean 54 cm) |
| coverage | 97% | 97–98% when the run finishes |
| doubled walls | none in any run | **one completed run in seven** |

The corridor is the whole difference. **A real office at 15 cm lidar height
looks like the depot world**, not the corridor world — desks, chairs, cable
trays, partitions, features in every direction. Expect depot-like results in
the rooms, and corridor-like variance only in a long featureless hallway.

**Before you navigate against a map, open the `.pgm` and look at it.** Look for
two parallel walls a few tens of centimetres apart with free space between them,
and for a wall that is straight in the building but bent or doubled in the map.
This is the check nothing else replaces: in one simulated run the rover covered
95% of the building, drove 108 m through 31 narrow gaps, touched nothing — and
put the far wall a metre and a half inside where it really is, as a doubled
smear down its whole length. Coverage, distance, doorway count and the contact
count all looked healthy. Only the picture showed it.

## 7. When something goes wrong

| symptom | cause | do this |
|---|---|---|
| no `/map` | SLAM has no scan | `ros2 topic hz /scan_filtered`. If 0, the box filter is not running or `/scan` is absent |
| map has doubled walls | a false loop closure, or a wrong lidar mount | re-measure the lidar transform; re-map driving a loop; do **not** lower the loop-closure acceptance thresholds |
| pose jumps between two positions | two publishers of `odom -> base_footprint` | §4 |
| rover will not move, nothing logs an error | `velocity_smoother` never left UNCONFIGURED, so the chain is dead at the top | `ros2 lifecycle get /velocity_smoother` — must be `active`. It is a lifecycle node and needs a manager; `real_mapping.launch.py` handles it |
| rover will not move | the velocity guard is holding it | `ros2 topic echo /diagnostics`, or the guard's own log line: `command_stale` means nothing is publishing `/cmd_vel_nav`; `scan_stale` means the lidar stopped |
| autonomous run stops and never restarts | the known stall, §1 | Ctrl-C the explorer, drive clear by hand, relaunch the explorer. Do not wait it out |
| rover stops near obstacles and will not continue | collision monitor StopZone (0.26 m box) | back it off manually; the zone is a hard stop by design |
| markers detected but in the wrong place | `marker_length` wrong, or SLAM drift | §2 — measure the black square. To tell the two apart, run `score_aruco.py --samples <csv> --marker-length <m>` against markers at known positions: it reports the marker length your data implies |
| markers not detected at all | too small, too far, too shiny, or the wrong dictionary | markers must be ≥ ~18 px in the image: 15 cm at 640×480 with a 60° FOV is detectable to about 4.5 m. Check `/aruco/debug_image` with `publish_debug_image:=true` |
| map drifts along a corridor | expected; the corridor is longer than the lidar | drive a loop, §6 |

## 8. The safety chain was tested end to end

Not "the nodes start" — the actual behaviour, against the **real** profile's
own configs, driven by a synthetic rover (`scripts/fake_rover.py` publishing
`/scan`, `/wheel_odom` and the two transforms the guard requires):

| | result |
| --- | --- |
| `/cmd_vel_nav` 0.08 m/s, nothing nearby | `/cmd_vel` = 0.08 m/s |
| obstacle at 0.50 m (outside the 0.34 m slowdown box) | `/cmd_vel` = 0.08 m/s, unchanged |
| obstacle at 0.20 m (inside the 0.26 m stop box) | **`/cmd_vel` stops** |
| obstacle removed | `/cmd_vel` = 0.08 m/s again |

So the command really does traverse velocity_smoother → velocity_guard →
collision_monitor, and the monitor really does stop the rover.

Two details worth knowing:

- On a stop the monitor publishes a zero Twist and then **stops publishing**.
  The rover firmware's own `cmd_vel` watchdog is the backstop that keeps it
  stopped. Confirm that watchdog exists before relying on this.
- `velocity_smoother` is a **lifecycle** node. Started without a manager it
  sits in UNCONFIGURED, publishes nothing, and the entire chain is dead from
  the top with no error anywhere. `real_mapping.launch.py` manages it; if you
  assemble your own launch, do the same, and check with
  `ros2 lifecycle get /velocity_smoother`.

You can repeat this test on the bench with no rover attached:

```bash
python3 scripts/fake_rover.py &
ros2 launch leo_nav2_exploration real_mapping.launch.py
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist '{linear: {x: 0.08}}'
ros2 param set /fake_rover front_range 0.20   # should stop /cmd_vel
ros2 param set /fake_rover front_range 0.0    # should resume
```

## 9. What has not been tested on hardware

Being explicit, because a first run goes better when you know where to look:

- **Everything in this document has been validated in simulation only.** Zero
  contacts and zero near-misses in every simulated run, but simulation has no
  unmodelled wheel slip, no reflective glass, no people.
- **The IMU bridge has never seen a real `leo_msgs/Imu`.** Topic name, rate and
  noise are assumptions.
- **The RealSense point cloud rate on the rover's computer is unknown.** In
  simulation it runs at 5 Hz and that is enough for the costmap. If the rover's
  CPU cannot sustain it, drop the camera obstacle source before dropping
  anything else — the lidar is what builds the map.
- **The 12 m laser range is a datasheet number.** Confirm real returns at that
  distance before trusting long-corridor behaviour.
- **A real office at 15 cm lidar height is much more cluttered** than the
  simulated worlds: chair legs, cable trays, table feet. Expect a busier map
  than the simulation figures suggest.
