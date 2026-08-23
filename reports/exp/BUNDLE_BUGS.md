# Bugs found in `leo_nav2_exploration_bundle.zip`

All seven were found by actually launching the overlay against the existing
`leo1` Gazebo simulator. `validate_bundle.sh` passes and 73/77 of the bundle's
own unit tests pass with every one of these present — none of them are
reachable by static validation or by the bundle's test suite, because all seven
only manifest at ROS 2 launch/runtime.

Reading order: bugs 1–4 are the startup killers, ordered by where they bite.
Bugs 5–7 are documented after the "not bugs" section — 5 is the doorway
regression's broken integration contract, 6 is the costmap raytracing failure,
and 7 is the controller that never commands forward motion. Numbers 6 and 7 are
the two that kept the rover immobile after 1–4 were fixed, and neither of them
crashes anything, which is what made them slow to find.

Fixes are applied in **both** `src/leo_nav2_exploration/` (the built copy) and
`bundle_ref/` (the pristine extraction), and to **both** the `sim` and `real`
profiles where the config is shared, since the `real` profile is what goes on
the rover.

---

## 1. `@dataclass` in a launch file aborts the whole launch — FATAL

`launch/navigation_overlay.launch.py`

```
AttributeError: 'NoneType' object has no attribute '__dict__'
  File "/usr/lib/python3.10/dataclasses.py", line 711, in _is_type
    ns = sys.modules.get(cls.__module__).__dict__
```

ROS 2's launch loader `exec`s the launch file as a module it never registers in
`sys.modules`. `dataclasses` looks the owning module up by name while building
each class and dies on the `None`. Nothing launches at all.

**Fix**: register a proxy module sharing the file's namespace before the first
`@dataclass` runs.

**Why the tests missed it**: `test_launch_source_contracts.py` imports the file
as a normal Python module, where `sys.modules` is populated.

## 2. Costmap `width`/`height` are floats — FATAL

`config/{sim,real}/nav2.yaml:166-167`

```
terminate called after throwing an instance of
  'rclcpp::exceptions::InvalidParameterTypeException'
  what(): parameter 'height' has invalid type: ...
           parameter {height} is of type {integer},
           setting it to {double} is not allowed.
```

Nav2 Humble declares the rolling-window costmap `width`/`height` as `int`
metres. `4.0` aborts `controller_server` during costmap construction, so there
is no local costmap and no controller.

**Fix**: `width: 4`, `height: 4`.

## 3. `tf_message_filter_target_frame` crashes the scan filter — FATAL

`config/{sim,real}/scan_filter.yaml`

```
terminate called after throwing an instance of
  'tf2_ros::CreateTimerInterfaceException'
  what(): timer interface not set before call to waitForTransform
```

`laser_filters` 2.0.x builds its tf2 message filter without installing a timer
interface, so setting a target frame aborts the node. `scan_to_scan_filter_chain`
dies, `/leo1/scan_filtered` never exists, and SLAM — which the bundle points at
the *filtered* topic — gets no scans at all. No map.

**Fix**: drop `tf_message_filter_target_frame` and
`tf_message_filter_tolerance`. `LaserScanBoxFilter` resolves
`scan_frame -> box_frame` itself, so the self-filter keeps working.

## 4. `velocity_guard` dies on its first state change — FATAL, and the worst one

`leo_nav2_exploration/velocity_guard_node.py:159`

```
ValueError: Logger severity cannot be changed between calls.
```

```python
level = self.get_logger().info if decision.permitted else self.get_logger().warning
level(f"velocity guard state: {decision.reason}")
```

rclpy caches logging metadata per *call site* and rejects the same site being
used at two severities. The node survives startup and dies the moment the guard
first flips between permitted and blocked.

This is the dangerous one, because of how it fails. The guard sits mid-chain:

```
controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
  -> velocity_guard -> cmd_vel_guarded -> collision_monitor -> cmd_vel
```

When it dies, `cmd_vel_guarded` goes silent, the collision monitor has no input,
and `/leo1/cmd_vel` stops. Everything upstream looks perfectly healthy —
frontier goals are dispatched and accepted, `cmd_vel_nav` publishes at 10.7 Hz,
Nav2 reports no errors — and the rover simply never moves. Observed directly:
ground-truth pose pinned at `(0.9918, 0.0000)` for the entire run while the
explorer cheerfully re-dispatched the same frontier seven times.

**Fix**: use two distinct call sites, one per severity.

---

## Not bugs (checked and cleared)

- **`preflight_check` FAILs on `cmd_vel_nav` publishers.** It expects exactly
  one, but `behavior_server` legitimately publishes recovery motions there — the
  bundle's own integration doc requires recoveries to pass through the same
  chain. The check is too strict; the graph is correct.
- **`preflight_check` FAILs on `map <- leo1/base_link`** with an extrapolation
  error. A startup race against sim time, transient.
- **`rosdep` cannot resolve `ament_python`.** Harmless `package.xml` wording;
  colcon builds the package fine.
- **Camera point cloud appears dead.** The Gazebo `rgbd_camera` renders lazily
  and the `ros_gz` bridge only subscribes upstream once a ROS subscriber
  exists, so `/leo1/camera/points` reads 0 Hz until the consuming stack is up.
  With the overlay running it publishes at ~4.2 Hz. Measure it *after* starting
  the stack, never before.

## Missing system dependency

`ros-humble-laser-filters` is required by the overlay but is not in the base
image, and `rosdep` could not install it until `apt-get update` was run
(stale package lists). `ros-humble-diagnostic-updater` also had to be upgraded
to 4.0.7 — the 4.0.6 in the image did not provide the `libdiagnostic_updater.so`
that `laser_filters` 2.0.9 links against. Both are baked into the
`leo_rover_humble:bundle` image.

---

## 5. The doorway regression cannot run against this repo — integration contract violated

`launch/sim_doorway_regression.launch.py` includes **`two_robots.launch.py`**
and passes:

```python
launch_arguments={
    'world': 'empty',
    'leo1_pose': '-1.50,0.0,0.20,0.0,0.0,0.0',
    'leo2_pose': '20.0,20.0,0.20,0.0,0.0,0.0',
}
```

`two_robots.launch.py` in this repository **declares no launch arguments at
all**. It hardcodes:

```python
num_robots = 1
world_path = os.path.join(ws_root, 'src', 'husarion_gz_worlds', 'worlds',
                          'husarion_office.sdf')
```

So all three arguments are silently discarded. The regression would spawn the
two-room doorway fixture at the origin *inside the furnished husarion office*,
intersecting existing geometry, with the rover wherever that launch happens to
put it — nowhere near the 1.5 m standoff `doorway_goals.yaml` assumes. There is
also no `empty.sdf` in either world directory; the closest is
`empty_with_plugins.sdf`.

The bundle's `test_launch_source_contracts.py` cannot catch this: it only reads
the bundle's own files and never checks the signature of the launch file it
depends on.

**Fix**: added `launch/sim_doorway_regression_leo.launch.py`, which includes
`two_robots_gpu.launch.py` (that one *does* honour `world`, `gui`,
`num_robots`, `enable_camera`, `gt_odom_tf`) against `empty_with_plugins`, and
spawns the fixture at `x = +1.5` so the door sits 1.5 m ahead of where leo1
actually spawns (the world origin). The scenario geometry in
`doorway_goals.yaml` then holds without editing it.

---

## 6. VoxelLayer feeds the local costmap a frozen sensor origin — silently disables raytracing

Observed continuously in `bundlerpp_office_world_realistic`:

```
[local_costmap.local_costmap]: Sensor origin at (-0.80, 2.35 0.20) is out of
map bounds (-0.83, 10.00, 0.00) to (3.16, 13.99, 1.18).
The costmap cannot raytrace for it.
```

Evidence gathered at the moment it was firing:

| quantity | value |
| --- | --- |
| live TF `leo1/odom -> leo1/sensor_camera_link` | `(1.064, 11.753, 0.200)` |
| rolling-window centre (correct, tracks robot) | `(1.165, 11.995)` |
| origin the layer reported | `(-0.80, 2.35, 0.20)` |
| measured `/leo1/camera/points` latency | **0.20 s** |
| measured `/leo1/scan_filtered` latency | 0.11–0.30 s |

The z is right and the xy is stale, and the 3-D bounds (z 0 → 1.18, i.e.
`z_voxels: 24` × `z_resolution: 0.05`) identify the reporter as the VoxelLayer
rather than the scan ObstacleLayer. Cloud latency of 0.20 s rules out the
obvious "the point cloud is lagging" explanation.

Consequence: raytracing is disabled, so **nothing is ever cleared** from the
local costmap. Obstacles accumulate until the rover is boxed in by cells that
no longer correspond to anything real, and the planner then legitimately
reports no valid path. This is the mechanism behind the late-run planner
failures that survived the planner tuning.

**Fix**: move the camera to a plain `ObstacleLayer` PointCloud2 observation
source. This repo's own `nav2_params_leo.yaml` uses exactly that wiring for the
same `/leo1/camera/points` topic and produced a 95.8%-coverage run with zero
such warnings, so it is known-good on this simulator. `clearing: true` is kept
(the repo config uses `clearing: False`, which lets depth false-positives
accumulate until they scroll out of the rolling window).

**Verified**: the doorway regression after this change logs
`out of map bounds` **0 times** and `failed to create plan` **0 times**.

---

## 7. DWB never commands forward motion on this robot

Not a crash, so nothing flags it — the stack looks completely healthy while the
rover sits still.

Measured in `bundle_office_world_realistic`, with the rover in **0.632 m of
open space** (ground-truth clearance):

```
/leo1/cmd_vel_nav       linear.x = 0.0   angular.z = 0.04
/leo1/cmd_vel           linear.x = 0.0   angular.z = 0.0138
```

`0.0138` is not arbitrary: with `max_vel_theta: 0.4` and `vtheta_samples: 30`,
the sample step is `0.8/29 = 0.0276`, and index 15 is `0.0138`. DWB is choosing
**the velocity sample nearest zero** — every genuine trajectory scores worse
than standing still. The critic balance is the cause:

| critic | scale |
| --- | --- |
| PathAlign | 28.0 |
| PathDist | 28.0 |
| RotateToGoal | 24.0 |
| GoalAlign | 20.0 |
| GoalDist | 20.0 |
| **ObstacleFootprint** | **0.03** |

Obstacle awareness is effectively switched off in trajectory scoring (all
collision avoidance is delegated to the Collision Monitor), while five
path/goal critics fight each other.

Compounding it, `rotate_to_goal_heading: true` with `yaw_goal_tolerance: 0.2`
and `RotateToGoal.slowing_factor: 5.0` meant that on arriving at each frontier
the rover began a rotate-to-heading at 0.0138 rad/s — a 90° turn would take two
minutes — to satisfy a heading that is meaningless for exploration.

**Fix**: replaced DWB with `RegulatedPurePursuitController` behind the same
`RotationShimController`. RPP is Nav2's recommended controller for a
differential-drive robot in tight indoor space: it has a handful of physically
meaningful parameters instead of a seven-critic balance, and it slows for
curves and obstacle proximity by construction.

**Measured effect on `office_world`**, same world/sensors/odometry/cap:

| | DWB (as shipped) | RPP |
| --- | --- | --- |
| coverage | 24.3% | **66.6%** |
| path length | 4.29 m | **55.31 m** |
| doorway passes | 5 | **12** |
| planner failures | 6 | **0** |
| contacts | 0 | 0 |

**Left alone but worth raising before hardware**: `ObstacleFootprint` at
`scale: 0.03` means the controller itself does essentially no obstacle
avoidance. That survived simulation because the Collision Monitor is a good
backstop, but on a real rover I would not want the only thing between the robot
and a wall to be a reactive stop.

---

## Addendum: the VoxelLayer defect, diagnosed

Bug 6 was originally worked around rather than understood. It has now been
re-tested deliberately and root-caused far enough to be actionable.

**The observation origin is frozen, not stale.** Across a full run the layer
emitted 6,416 `out of map bounds` warnings containing **exactly one distinct
origin**, `(0.65, 0.62, 0.20)` — approximately where the rover started —
while the live TF `leo1/odom -> leo1/sensor_camera_link` read
`(-5.105, -2.573, 0.200)` and the rolling window was correctly centred on the
robot at `(-5.005, -2.305)`. The observation buffer captured one origin at
startup and never replaced it, so raytracing is disabled for the entire run and
nothing ever clears.

Two plausible explanations were tested and **falsified**:

| hypothesis | test | result |
| --- | --- | --- |
| large odometry drift moves the rolling window away from the observation origin | re-ran with the EKF, odometry error 0.79 m instead of ~11 m | warnings still appeared — **not drift** |
| the point cloud's timestamps are frozen, so the TF lookup always returns the same early pose | measured cloud header stamps against `/clock` | stamps advanced 13.4 s over a 16 s probe, lag 0.2–0.8 s — **not the stamps** |

It is **VoxelLayer-specific**: an ObstacleLayer consuming the *same*
`/leo1/camera/points` topic, same TF tree, same `ObservationBuffer` code path,
produces **zero** such warnings across every run.

**Measured cost**, A/B with identical seed, identical EKF, camera layer the only
difference:

| | VoxelLayer | ObstacleLayer |
| --- | --- | --- |
| `out of map bounds` warnings | **6,416** | **0** |
| phantom walls | 0.144 | **0.044** |
| wall RMSE (aligned) | 0.309 m | **0.076 m** |
| wall IoU (aligned) | 0.627 | **0.692** |
| SLAM ATE RMSE | 0.579 m | **0.295 m** |
| path driven | 78.0 m | **62.9 m** |
| coverage | 96.0% | 96.4% |
| contacts / near-misses | 0 / 0 | 0 / 0 |

Note the failure is *degrading*, not catastrophic — no collisions, the same 21
narrow-gap transits — but the map is materially worse and the rover drives 24%
further to cover the same ground.

**Conclusion.** Keep the camera on an ObstacleLayer. This now rests on a
diagnosis and a controlled comparison, not on expedience. VoxelLayer remains the
theoretically better structure for a depth camera (3-D accumulation and
clearing), so it is worth retrying on hardware with a newer Nav2 — but on this
version it is broken in a way configuration cannot fix.
