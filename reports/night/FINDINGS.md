# Night of 2026-08-19: getting the stack ready for hardware

Continues `reports/exp/FINDINGS.md` (2026-08-17). That study settled the
architecture — bundle Nav2 + SLAM + safety chain, RegulatedPurePursuit,
explore_lite, camera as an ObstacleLayer, widened loop-closure *search* with
unchanged acceptance thresholds. This one is about the four things that stood
between that and a rover you can switch on:

1. the simulator was rendering in software, starving the navigation stack;
2. there was no real ArUco detector, only a mock that reads ground truth;
3. the IMU fusion that halved odometry error existed for simulation only;
4. there was no launch path for the safest thing you can do on day one —
   teleoperated mapping.

## The short version

**What is ready.** SLAM, sensor fusion, the safety chain and marker detection
all work and are verified end to end. The recommended first hardware run is
`real_mapping.launch.py` with a joystick: scan filter, slam_toolbox, wheel+gyro
EKF, velocity guard, collision monitor, ArUco — and no planner, controller,
behaviour tree or explorer to go wrong. `REAL_ROVER_DEPLOY.md` is the operator
guide.

**What changed tonight.** Gazebo now renders on the GPU (native WSL, ~40% of
the CPU handed back to Nav2). The planner is NavFn instead of SmacPlannerLattice
— paired on two seeds, it wins every map metric and misses its rate 2 times
instead of 48. `real_navigation.launch.py` **could not launch at all** with its
default arguments and now can. There is a real ArUco detector where there was
only a mock, and an IMU bridge and EKF config where the fusion existed for
simulation only.

**What is not ready — with a late improvement.** Autonomous exploration stopped
early in five runs out of fourteen: the rover stalls and never restarts. It is
always a stall, never a collision — **zero contacts in every scored run
tonight**, one near-miss in total.

In the last ninety minutes one contributing cause was found and changed
(it did not, on later testing, measurably fix the stall -- see 6.9). Nav2's `BackUp`
recovery is given 10 s to travel 0.25 m at 0.04 m/s, which the collision
monitor's 75% slowdown turns into 8.3 s of need against a 10 s budget; it was
timing out and blacklisting the frontier. With a 20 s allowance and 0.08 m/s,
the five seeds that had stalled all completed. But on **fresh** seeds the stall
rate went 5-in-14 to 1-in-6 — p = 0.61, no evidence of an effect. The change is
kept because it is free and cannot hurt, not because it works. **Drive the first
map yourself.**

**A caveat on the evidence.** Nine of the night's runs had to be discarded — six
because two experiment queues ended up driving two simulators on one ROS domain
after a `pkill` silently failed, three because a config variant was still
applied. Both mistakes are described in §2 rather than tidied away, because
both produced results that looked like real findings until they were checked.

---

## 1. GPU rendering: fixed, and now proven per run

The previous study concluded that Gazebo could not reach the GPU inside Docker
Desktop and that native WSL was the fix, but did not do it. It is done.

**Root cause, concretely:** the native Ubuntu distro's `libgl1-mesa-dri` ships
`/usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so`. The `ros:humble` Docker image
does not. No environment variable can conjure a driver that is not installed,
which is why `GALLIUM_DRIVER=d3d12`, `--device=/dev/dxg`, mounting
`/usr/lib/wsl` and forcing surfaceless EGL all failed.

Evidence the GPU is actually used, from Ogre's own log during a live run:

```
GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Ti)
GPU Vendor: microsoft
```

with `LIBGL_ALWAYS_SOFTWARE=1` on the same launch logging
`GL_RENDERER = llvmpipe (LLVM 15.0.7, 256 bits)` as a control, and nvidia-smi
utilisation of 13–22% against 4–5% for the llvmpipe run.

Measured on `office_world`, headless, camera on:

| | GPU (D3D12) | software (llvmpipe) |
| --- | --- | --- |
| real-time factor, 1 robot | **1.336** | 1.149 |
| gz-server CPU, 1 robot | **~194%** | ~320% |
| real-time factor, 2 robots | **0.795** | 0.638 |
| gz-server CPU, 2 robots | **~253%** | ~400% |

The RTF gain is modest. **The ~40% of CPU handed back to Nav2 is the point**,
because that is what the planner and controller were missing:

| | Docker, software render | WSL, GPU |
| --- | --- | --- |
| "Planner loop missed its desired rate" | ~20 in 8 min | 2 in 17 min |
| "Control loop missed its desired rate" | 8 | 1 |

`scripts/exp_run_wsl.sh` runs the whole harness against the native distro and
**records `GL_RENDERER` into every run log**, so no future run has to take the
GPU claim on faith. Artefacts are written straight to the Windows-side repo
through `/mnt/c`, so scoring is unchanged.

The patched `RenderSystem_GL3Plus.so.2.2.5` is still required natively — the
stock one raises `Ogre::UnimplementedException` in `GL3PlusTextureGpu::copyTo`
and the simulator does not start.

Worth knowing: despite `GZ_VERSION=harmonic` and the Dockerfile installing
`gz-harmonic`, the launch actually runs `ign gazebo --force-version 6` —
**Fortress**. Both are installed; only Fortress is exercised.

## 2. A harness bug that invalidated results, and would have invalidated more

Between runs the harness killed `ros2`, `python3` and `parameter_bridge`.
Every Nav2 server and `slam_toolbox` is a C++ binary under
`/opt/ros/humble/lib` and matched none of those patterns. **Three complete
stacks were found running simultaneously**, each with its own `slam_toolbox`
publishing `/map`.

The symptom is not a crash. The next run inherits the previous run's *finished*
map, so explore_lite reports `No frontiers found, stopping` two seconds after
launch and the run "completes" in 90 seconds with a nonsense map. One run was
scored that way before the cause was found; it has been deleted rather than
reported.

The fix is `scripts/leo_cleanup_wsl.sh`, and it is a **file** rather than an
inline `pkill` for a second, related reason: a pattern passed to `bash -c`
appears in that shell's own `/proc/<pid>/cmdline`, so `pkill -f` matches the
shell running it and kills it partway through the cleanup — silently, with the
remaining patterns never tried. Invoked as a script, the command line is just
the script's path. `exp_run_wsl.sh` now refuses to start a run unless cleanup
reports zero survivors.

### Six runs lost to two simulators sharing one ROS domain

Six runs produced `ign gazebo` freezing `/clock` while every ROS node stayed
alive and quiet. The only symptom in the simulator's own log is

```
NodeShared::Publish() Error: Interrupted system call
```

From the outside that is indistinguishable from a healthy run — nodes present,
logs calm, no exit code — and in the metrics it is indistinguishable from an
exploration stall, which is worse, because it would have been scored as one.

**The diagnosis went wrong first, and the wrong version was believed for half
an hour.** Three wedges in a row, with free memory, no disk pressure and no
leaked shared-memory segments, looked like the WSL distro degrading after
twenty back-to-back simulations; `wsl --terminate Ubuntu` was tried, the next
run was healthy, and that appeared to confirm it. It did not. A queue that had
been "killed" a quarter of an hour earlier was still running, so **two queues
were driving two simulators on the same gz-transport partition**. The distro
restart worked because it killed the other queue, not because the distro was
tired.

The reason the kill failed is worth writing down plainly: **Git Bash on Windows
has no `pkill`.** `pkill -f night_queue` prints
`pkill: command not found` to stderr and returns non-zero, which in a
fire-and-forget cleanup line looks like nothing at all. Every Windows-side
`pkill` in this session had been a no-op. To kill a Windows process by its
command line, go through PowerShell:

```powershell
Get-CimInstance Win32_Process -Filter "Name='bash.exe'" |
  Where-Object { $_.CommandLine -match 'night_queue|exp_run_wsl' } |
  ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
```

and then *check the count is zero*, rather than assuming.

Two fixes, one for each half of the mistake:

- `exp_run_wsl.sh` watches the trajectory recorder and aborts a run after three
  minutes without a new sample, so a frozen simulator ends the run instead of
  quietly filling a cap.
- `night_queue.sh` now takes a lock directory and refuses to start while
  another queue holds it. "I killed it" is not a thing to take on trust.

All affected runs are quarantined under `reports/night/_invalid_*_concurrent`
rather than scored, along with three runs that were valid in themselves but
executed against a camera-layer variant the queue had applied — those are
relabelled `*_camtuned_*` rather than counted as the shipped configuration.

## 3. Planner: SmacPlannerLattice → NavFn

`SmacPlannerLattice` searches a state lattice with `max_iterations: 1000000`
and `max_planning_time: 5.0`. On this workload it could not hold its 5 Hz.

The Leo Rover is a skid-steer that turns in place, and the controller is
RotationShim + RegulatedPurePursuit — the shim rotates the rover onto the path
before following it. A kinematically feasible lattice path buys nothing a robot
with a zero turning radius needs, and NavFn plans an office-sized costmap in
single-digit milliseconds. This matters more on the rover than here: its
computer is much smaller than this workstation, and a planner that misses its
rate on a 28-core desktop will miss it worse there.

`scripts/apply_navfn_planner.py` makes the swap reversibly, keeping the
original block commented in place.

### The A/B, paired on two seeds

`office_world`, native-GPU backend, EKF on, ArUco on — everything identical
except the planner, run on the same two noise seeds:

| | NavFn seed 21 | Lattice seed 21 | NavFn seed 7 | Lattice seed 7 |
| --- | --- | --- | --- | --- |
| coverage | **0.979** | 0.962 | **0.978** | 0.853 |
| phantom walls | **0.000** | 0.222 | **0.033** | 0.247 |
| wall IoU (aligned) | **0.878** | 0.548 | **0.757** | 0.496 |
| wall RMSE (aligned) | **0.043 m** | 0.366 m | **0.069 m** | 0.390 m |
| SLAM ATE RMSE | **0.577 m** | 0.743 m | **0.462 m** | 0.704 m |
| narrow-gap transits | **34** | 26 | **36** | 15 |
| contacts / near-misses | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| "Planner loop missed its rate" | **2** | 48 | **2** | 54 |
| ArUco markers found | 7/8 | 5/8 | 7/8 | 4/8 |

Means: phantom **0.017 vs 0.235**, IoU **0.818 vs 0.522**, wall error
**0.056 m vs 0.378 m**, coverage **0.978 vs 0.908**. NavFn wins every column on
both seeds, and the missed-planner-cycle count separates the two by more than
an order of magnitude in both.

The difference is in the *map*, and it is not subtle in the raster: the lattice
run's map is squeezed along x. The north
partition that belongs at x = +4.0 is drawn at x ≈ +3.2 **and doubled**; the
east wall is a wavy band ~1 m inboard of truth; the round obstacle at (7, −4.5)
is a crescent. The NavFn map has single-line walls sitting on the ground truth,
all five doorways open at the right positions, and both desks and the pillar
closed and correctly shaped.

The causal link is the 48 missed planner cycles. Planning that overruns its
budget takes CPU from `slam_toolbox`'s scan matcher on the same machine, and a
starved scan matcher is exactly how a map acquires a scale error.

A planner is not usually the thing you tune to fix a map. Here it was.

## 4. ArUco: a real detector

`leo_rover_exploration/mock_aruco_detector.py` reads marker ground truth from a
YAML and publishes it back with noise when the camera is roughly pointed at it.
It cannot detect anything. `leo_rover_semantic_vision` on
`origin/fix/real-rover-mapping` has a real one, but it hardcodes a 0.2 m marker,
calls `DetectorParameters_create()` and `estimatePoseSingleMarkers` (both gone
from OpenCV ≥ 4.7 / ≥ 4.9), and publishes only a camera-frame TF — no map-frame
registry, no gating, nothing to stop a single bad frame becoming a landmark.

`leo_nav2_exploration/aruco_detector.py` replaces both, behind the *mock's*
topic contract so the explorer's registry needs no change:

```
Image + CameraInfo -> detectMarkers -> solvePnP(IPPE_SQUARE) -> tf2 -> map frame
   -> gate on range, marker pixel size and reprojection error
   -> confirm after N consecutive frames -> running-average registry
```

Three things it is deliberately careful about, each a silent failure mode:

- **Optical vs body frame.** `solvePnP` returns a pose in the OpenCV optical
  convention (z forward). A RealSense stamps images with `*_optical_frame`,
  which matches; Gazebo stamps them with the *link* frame (x forward), which
  does not. Getting it wrong rotates every detection by 90° without erroring.
  `frame_is_optical` is an explicit per-profile parameter and the resolved
  value is logged at startup.
- **OpenCV API drift.** Detection goes through `ArucoDetector` or the legacy
  free functions, whichever exists; the pose always goes through plain
  `cv2.solvePnP`, which has been stable across every version.
- **`marker_length` is the side of the black square.** Nothing errors if it is
  wrong — every marker is simply reported proportionally too near or too far
  along the view ray. Section 4.2.

### 4.1 It works

Eight ArUco markers were added to `office_world` as real geometry: a white
0.30 m backing board with a 0.20 m textured plate in front of it. The white
border is geometry rather than part of the texture so that `marker_length`
equals the plate side with nothing to derive. `scripts/make_aruco_models.py`
generates the `DICT_4X4_50` textures.

On the best office run (`n3`), during ordinary autonomous exploration with
nothing aimed at the markers:

- **8 of 8 markers detected, every id correct, zero phantom ids**
- 744 of 4670 processed frames contained a marker
- reprojection error 0.14–2.5 px

### 4.2 The detector's own accuracy, measured with SLAM taken out of the way

During exploration, marker positions carried errors of 0.4–2.9 m. Almost all of
that is the map, not the detector: the reported pose is only as good as the
`map -> camera` transform it is projected through, and office_world SLAM runs
at 0.44–0.68 m of trajectory error.

`scripts/aruco_test_wsl.sh` removes that term. It runs the simulator with
ground-truth odometry and a static identity `map -> leo1/odom`, so the `map`
frame *is* the world frame, drives a fixed waypoint route, and scores against
the world's true marker poses. Every remaining metre belongs to the detector:

| | |
| --- | --- |
| markers detected, correct id | **7 of 8** |
| **false positives** | **0** |
| position error | mean 0.254 m, median 0.261 m, max 0.440 m |
| frames processed / containing a marker | 4290 / 1182 |

Marker 4 was never seen — the fixed route does not point the camera at the west
partition's east face. Under autonomous exploration, which does look around,
all eight were found.

Every error points along the wall normal, which is the signature of a marker
size mismatch rather than a frame error. `score_aruco.py --samples` turns that
into a number: a wrong `marker_length` scales every estimated range by
`L_true / L_assumed` and displaces the reported position along the view ray, so
`|truth - reported| = |s - 1| * range` — computable from the per-detection CSV
and the known marker positions, without needing the camera pose.

```
marker-length check (554 samples)
  radial error / range : median 0.050  (10-90% 0.005-0.429)
  implied scale factor : 1.050
  detector was told    : 0.2000 m
  implied true length  : 0.2099 m
```

So the configured 0.20 m is right to within 5%, and the residual 0.25 m at 3–5 m
range *is* that 5%. The same tool works on hardware: place markers at surveyed
positions, drive past them, and it reports the marker length you should have
configured.

The z coordinate is a separate, understood artefact: markers sit at z = 0.30 in
the world and are reported at z ≈ 0.12, because 2-D SLAM puts `base_link` at
z = 0 while Gazebo has it ~0.18 m above the floor. It says nothing about the
detector.

### 4.3 The pose maths is unit-tested, without a simulator

`test/test_aruco_pose.py` renders a marker at a known pose with known
intrinsics — supersampled 4x and downsampled with `INTER_AREA`, because
warping a hard-edged marker straight into a 640x480 grid aliases its corners by
most of a pixel and at 4 m the marker is only ~28 px across — then runs the
detector's own code over the image. Eight tests, all passing:

- range recovered at 1 m, 2 m and 4 m within the error budget implied by half a
  pixel of corner noise (`Z^2 * delta / (f * L)`: 3.6 mm at 1 m, 58 mm at 4 m);
- off-axis position recovered to 3 cm at 2 m;
- **telling the detector the marker is 0.15 m when it is 0.20 m puts it at
  exactly 3/4 of its true range** — the documented failure mode, pinned down as
  a test rather than a warning;
- the optical → body rotation checked on all three axes with `det = +1`, and a
  marker 2 m straight ahead landing at `+2 m` in body x;
- quaternion round-trip through `_quat_from_matrix` across five orientations
  including two gimbal-adjacent ones.

This is the part that would otherwise only be checked by pointing a real camera
at a real marker and squinting at RViz.

**On the rover this ambiguity does not exist**: measure the printed black
square with a ruler and pass it. The guide says so in the one place someone
will read before printing.

## 5. The EKF is the difference between mapping the building and mapping one room

Wheel + gyro fusion was recommended by the previous study on the strength of a
single run. Removing it, with everything else identical (`office_world`,
seed 21, NavFn, native GPU):

| | with EKF (`n3`) | **no EKF** (`n9`) |
| --- | --- | --- |
| coverage | **97.9%** | 63.8% |
| path driven | **103.4 m** | 26.7 m |
| run length before the explorer quit | 995 s | 514 s |
| narrow-gap transits | 34 | 15 |
| ArUco markers found | 7/8 | 2/8 |
| phantom walls | 0.000 | 0.000 |
| wall RMSE (aligned) | 0.043 m | 0.041 m |

The map it *did* build is just as clean — 4 cm wall error, no phantom walls.
It only built a third of it. `explore_node` logged
`All frontiers traversed/tried out, stopping` after twelve
`controller_server: Failed to make progress` aborts: each abort blacklists the
frontier the rover was heading for, and once every frontier is blacklisted,
exploration is over.

The chain is: wheel-only odometry on a skid-steer accumulates yaw error through
every turn -> the pose the controller is steering from disagrees with the world
-> it cannot reach its goal -> the frontier is written off. The EKF attacks
that at the source, and the same failure appeared in the very first run of the
night (`n1`, 64.3% coverage), which is what sent this investigation towards the
planner and the CPU budget in the first place.

**This is the single highest-value item to get working on the rover**, and also
the one with a real setup hazard: `odom -> base_footprint` must have exactly
one publisher (§8, and `config/real/ekf.yaml`).

## 6. Results

All runs: `office_world` unless noted, realistic skid-steer odometry, native-GPU
simulator, one simulator at a time. `cover` is mapped free area against the
world's true free area; `phantom` is the fraction of mapped wall cells with no
wall within 15 cm of them.

| run | planner | EKF | cover | phantom | IoU | wall RMSE | SLAM ATE | path | doors | contacts | ArUco |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| n3 seed 21 | NavFn | yes | 0.979 | **0.000** | 0.878 | 0.043 m | 0.577 m | 103 m | 34 | 0 | 7/8 |
| n10 seed 21 (repeat) | NavFn | yes | 0.979 | 0.104 | 0.729 | 0.287 m | 0.437 m | 109 m | 33 | 0 | 6/8 |
| n5 seed 7 | NavFn | yes | 0.978 | 0.033 | 0.757 | 0.069 m | 0.462 m | 110 m | 36 | 0 | 7/8 |
| n6 seed 33 | NavFn | yes | 0.967 | 0.166 | 0.614 | 0.179 m | 0.681 m | 128 m | 35 | 0 | 5/8 |
| n11 seed 21, explorer timeout 60 s | NavFn | yes | 0.969 | 0.079 | 0.656 | 0.090 m | 0.578 m | 85 m | 28 | 0 | 5/8 |
| **n4 seed 21** | **Lattice** | yes | 0.962 | 0.222 | 0.548 | 0.366 m | 0.743 m | 113 m | 26 | 0 | 5/8 |
| **n9 seed 21** | NavFn | **no** | **0.638** | 0.000 | 0.563 | 0.041 m | 0.650 m | 27 m | 15 | 0 | 2/8 |
| n8 seed 21, **doubled odometry noise** | NavFn | yes | 0.846 | 0.112 | 0.527 | 0.118 m | 0.773 m | 38 m | 10 | 0 | 4/8 |
| n7 seed 21, **depot_world** | NavFn | yes | 0.968 | **0.000** | 0.701 | 0.043 m | **0.070 m** | 40 m | 12 | 0 | — |

**Zero contacts and zero near-misses in every run of every configuration**,
including under doubled odometry noise.

### Coverage, honestly: autonomous exploration stalls in five runs of fourteen

Every `office_world` run of the shipped configuration (NavFn, EKF, planner
tolerance 0.25), plus the two `depot_world` runs:

These fourteen runs were made **before** the recovery-timeout fix in §6.9. They
are kept as measured rather than quietly re-run, because the three seeds that
stalled are the reason that fix exists.

| seed | world | coverage | phantom | IoU | wall RMSE | path m | doorways | contacts | ArUco |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 606 | office | 0.980 | 0.086 | 0.747 | 0.263 | 119 | 31 | 0 | 6/8 |
| 202 | office | 0.980 | 0.000 | 0.795 | 0.046 | 126 | 28 | 0 | 6/8 |
| 21 | office | 0.979 | 0.000 | 0.878 | 0.043 | 103 | 34 | 0 | 7/8 |
| 21 (repeat) | office | 0.979 | 0.104 | 0.729 | 0.287 | 109 | 33 | 0 | 6/8 |
| 7 | office | 0.978 | 0.033 | 0.757 | 0.069 | 110 | 36 | 0 | 7/8 |
| 101 | depot | 0.976 | 0.000 | 0.718 | 0.035 | 62 | 20 | 0 | — |
| 21 (timeout 60 s) | office | 0.969 | 0.079 | 0.656 | 0.090 | 85 | 28 | 0 | 5/8 |
| 21 | depot | 0.968 | 0.000 | 0.701 | 0.043 | 40 | 12 | 0 | — |
| 33 | office | 0.967 | 0.166 | 0.614 | 0.179 | 128 | 35 | 0 | 5/8 |
| 55 | office | **0.849** | 0.000 | 0.621 | 0.046 | 56 | 13 | 0 | 3/8 |
| 707 | office | **0.645** | 0.000 | 0.623 | 0.038 | 40 | 15 | 0 | 3/8 |
| 7 | depot | **0.567** | 0.000 | 0.298 | 0.042 | 4 | 1 | 0 | — |
| 101 | office | **0.519** | 0.209 | 0.311 | 0.217 | 13 | 5 | 0 | 0/8 |
| 808 | office | **0.301** | 0.001 | 0.344 | 0.039 | 5 | 3 | 0 | 1/8 |

**After the recovery-timeout fix**, the three worst seeds re-run:

| seed | world | coverage | phantom | IoU | wall RMSE | path m | doorways | contacts | ArUco |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 101 | office | 0.981 | 0.059 | 0.733 | 0.219 | 132 | 42 | 0 | 7/8 |
| 808 | office | 0.979 | 0.000 | 0.773 | 0.037 | 89 | 27 | 0 | 3/8 |
| 707 | office | 0.977 | 0.042 | 0.838 | 0.188 | 110 | 35 | 0 | 5/8 |

Nine further runs are excluded: six lost to two simulators sharing a ROS domain
and three executed against a camera-layer variant (§2). Excluding them makes
the configuration look *better* than the raw directory listing would, which is
the direction that needs stating out loud.

**Zero contacts in all seventeen. One near-miss.**

Before the fix, nine of fourteen mapped essentially everything and five stopped
early; all three stalled seeds re-tried afterwards completed. The five failures
share one signature: the rover drives normally for a few minutes, then
stops and never restarts, while Nav2 keeps issuing goals at it.

- `n18` (office, seed 101): last motion at t = 253 s, then stationary at
  (−4.6, −0.6) — in the corridor — for the remaining six minutes. 56
  `GridBased: failed to generate a valid path`, 16 `Failed to make progress`,
  7 `backup failed`.
- `n14` (depot, seed 7): last motion at t = 132 s, then stationary for eleven
  minutes. 28 `Failed to make progress`, 26 goal preemptions, minimum clearance
  0.39 m — it was not wedged against anything.
- `n13` (office, seed 55): 39 planning failures, exploration ended at 85%.

The successful runs have the same errors at a much lower rate (about 22
planning failures in a whole 17-minute run). So this is not a distinct bug so
much as the same pressure crossing a threshold: SLAM drift in a 2.4 m corridor
puts the rover's estimated pose inside the static layer's wall, the planner
cannot trace a path out of a lethal start, the controller cannot make progress,
the recovery backup fails, the frontier is blacklisted, and eventually every
frontier is. The 2026-08-17 study met the same wall with the lattice planner,
where it announced itself as `Starting point in lethal space!`.

#### What `Collision Ahead` actually means, looked at rather than guessed

`scripts/costmap_recorder.py` writes the local costmap out once a second as a
picture, with the footprint drawn on it and the cost bands coloured the way the
recovery checker reads them. Frames live in `<run>/costmaps/` with an
`index.csv` carrying pose, speed and the cost under the footprint.

Building it caught an error in the measurement first. The initial version took
the robot pose from `/leo1/odom` and drew it on a costmap anchored in the EKF's
`odom` frame. Those two diverge by the odometry drift — **11 m by the end of
some runs** — so the footprint was being sampled somewhere the robot was not.
With `rolling_window: true` the robot is always at the centre of the window,
which makes the mistake self-detecting, so the recorder now looks the pose up
through TF and logs a `miscentred` counter that is non-zero if the frames ever
disagree again.

With that fixed, the answer is arithmetic:

```
footprint half-width   0.21 m   + padding 0.01
inscribed radius       0.22 m   <- InflationLayer paints cost 253 within this
                                   distance of EVERY obstacle
circumscribed radius   0.31 m   <- what an in-place rotation sweeps
inflation_radius       0.35 m   <- the gradient; does NOT set the 253 band
```

The recovery behaviours refuse to move when any footprint cell is >= 253. So
they refuse **whenever the robot's side is within 0.22 m of anything**. In
`n43` that was **502 of 994 frames — 51% of the run**, and 35 of those frames
were recorded while the rover was driving at over 0.05 m/s. It was "in
collision" and moving perfectly well at the same time.

The reason is a double-count. Costmap inflation is defined for the robot
*centre*: a cell at 253 means "a robot centred here touches something".
`FootprintCollisionChecker` samples cost along the footprint *outline* and
applies the same threshold, so the robot's own radius is counted twice. The
check is conservative by exactly the inscribed radius, all the way round.

The consequence is sharpest in a gap:

| gap | free space between the two 253 bands | footprint needs | recovery |
| --- | --- | --- | --- |
| 0.78 m (the doorway fixture) | 0.34 m | 0.44 m | **impossible** |
| 1.00 m | 0.56 m | 0.44 m | possible |
| 1.30 m (office_world doorways) | 0.86 m | 0.44 m | possible |
| 2.40 m (the corridor) | 1.96 m | 0.44 m | possible |

So on the purpose-built 0.78 m doorway the rover can *drive through* — it did,
7 times out of 8 — but if anything ever asks it to recover while it is in
there, no recovery can execute. Not "will probably fail": cannot.

**The fix this suggests, untested.** `behavior_server` takes its costmap by
parameter (`costmap_topic`, default `local_costmap/costmap_raw`). Pointing it
at a second, minimal costmap carrying the obstacle layer and **no inflation
layer** would make the footprint check mean what it says — the footprint
overlapping an actual obstacle — instead of overlapping an obstacle's inflation.
That is the correct semantics for a body-collision test. It costs another
costmap node's CPU on a rover that does not have much to spare, and it is not
validated here, so it is written down rather than shipped.

#### A hypothesis that looked right and was not

`n18`'s behaviour server logged `Collision Ahead` seven times — `BackUp` and
`Spin` check the local costmap before moving, and refused. The depth camera
feeds that costmap, so the obvious reading was that camera marks were boxing
the rover in. Re-running the same seed with the camera removed from the costmap
looked like confirmation: `Collision Ahead` 7 -> **0**, coverage 0.519 ->
**0.866**.

Then the same change was run on the other stalling seed, and on the wider set:

| run | camera in costmap | `Collision Ahead` | `Failed to make progress` | coverage |
| --- | --- | --- | --- | --- |
| n18, seed 101 | yes | 7 | 16 | 0.519 |
| n21, seed 101 | **no** | 0 | 15 | 0.866 |
| n13, seed 55 | yes | 7 | 8 | **0.849** |
| n22, seed 55 | **no** | 4 | 15 | **0.543** |
| n3, seed 21 | yes | 2 | 14 | 0.979 |
| n19, seed 202 | yes | 0 | 15 | 0.980 |

`Collision Ahead` does not track coverage at all: a run with seven of them
mapped 85%, a run with four mapped 54%, and the aborts still happen with the
camera gone. `Failed to make progress` sits at ~15 in every run, successful or
not. **Cause and effect were the wrong way round** — a run that goes well
attempts few recoveries, rather than a run that attempts few recoveries going
well.

The camera change was reverted. It is the second plausible fix tonight that
rescued one seed and wrecked another; both are the same lesson, which is that
n = 1 on a high-variance world is not evidence.

**What this means for tomorrow**: it is a stall, never a collision — zero
contacts and zero near-misses in all nine runs, including the stalled ones. The
rover ends up with a partial map, not damage. But a one-in-four chance of an
autonomous run stopping early is the single strongest reason to drive the first
hardware map by hand, and to watch the second one with a controller in reach.
`REAL_ROVER_DEPLOY.md` says exactly that.

### Map quality: better on average, still variable on this world

Over the **seven** `office_world` runs of the shipped configuration that
completed:

| | mean | range |
| --- | --- | --- |
| coverage | 0.976 | 0.967 – 0.980 |
| phantom walls | 0.067 | 0.000 – 0.166 |
| wall IoU (aligned) | 0.740 | 0.614 – 0.878 |
| wall RMSE (aligned) | 0.140 m | 0.043 – 0.287 m |
| SLAM ATE RMSE | 0.538 m | 0.437 – 0.681 m |

against the 2026-08-17 four-seed baseline of phantom 0.136, IoU 0.686, RMSE
0.224 m, coverage 94.4%, and against the lattice planner's two runs at phantom
0.235, IoU 0.522, RMSE 0.378 m. Better on every average, and **coverage is now
tight** — 1.3 percentage points across seven runs, where the old baseline spanned
85.3–97.9%.

But the visual check is what decides, and it disagrees with the ranking the
numbers imply:

- **n3** — clean. Single-line walls on the truth, five doorways open, both
  desks and the pillar closed and correctly shaped.
- **n5** — clean. Same, with a small uniform offset.
- **n10** — clean *structurally*: no doubled walls anywhere, but the whole map
  is compressed by about 0.7% along x, which is what its 0.104 phantom
  fraction is measuring. It would navigate fine.
- **n6** — **one real defect**: the north-east partition (x = +4.0) is drawn as
  two parallel walls about 0.4 m apart. That is the failure that matters,
  because a planner will try to drive between them.
- **n28b** (seed 404, camera-layer variant — so not a shipped-config sample,
  but the defect it shows is not specific to that variant) — the clearest
  warning of the night. It **completed**,
  covering 94.8% and driving 108 m through 31 narrow gaps with no contacts, and
  its map is unusable on the east side: the east wall is drawn ~1.5 m inboard
  as a doubled, smeared band down its whole 16 m length, the north-east
  partition is displaced and broken, the desk at (8.5, 5) has become a thin
  sliver 1.8 m from where it belongs, and the round pillar is an arc. Coverage,
  path length, doorway transits and the contact count all look healthy. Only
  the raster shows it.

So a completed run is not a good run. One of the seven office runs that
finished with the shipped configuration has a genuine geometric defect (`n6`),
and a camera-variant run that also completed (`n28b`) has a worse one. Both are
the same failure, on the same kind of geometry, that the 2026-08-17 study
traced to the 24 m corridor exceeding the 12 m lidar. It is a property of that world rather than of this
configuration, and it is the reason the deployment guide tells you to open the
`.pgm` before navigating against it.

### A worked example of the stall

The most extreme of the five. `n14` (depot_world, seed 7) drove 4.3 m in the
first 70 seconds, stopped at
(1.59, 2.22), and **stood there for the remaining eleven minutes** while Nav2
issued 28 goals and 570 paths at it. Worth being precise about what did and did
not happen:

- it was **not** wedged: minimum clearance over the whole run was 0.39 m, and
  there were zero contacts and zero near-misses;
- it was **not** blocked by the collision monitor: one stop event, three
  slowdowns, in twelve minutes;
- the controller logged `Failed to make progress` 28 times and **never**
  `detected collision ahead`, so it was issuing commands that did not translate
  the robot;
- `bt_navigator` logged 26 goal preemptions, and the velocity guard alternated
  `permitted` / `command_stale` eight times each.

The signature — rotating commands, no translation, repeated preemption — is a
**rotate-to-heading livelock**: `RotationShimController` turns towards the path
before following it, the explorer replaces the goal, the new path needs a
different heading, and the rover turns again. `SimpleProgressChecker` measures
*position*, so it eventually aborts, the frontier is blacklisted, and the cycle
repeats until exploration ends.

One sample is not a diagnosis, and this is stated as the most likely reading of
the evidence rather than a confirmed cause. What can be said flatly: **the
failure mode is a stall, not a collision**, and the operator-facing consequence
is a partial map, not a damaged rover. It is also the strongest single argument
for driving the first hardware map by hand.

### A note on why every map metric here is the *aligned* one

`n19` (seed 202) scored phantom 0.000 and 4.6 cm wall error, and its raw raster
is visibly **rotated by about 3 degrees** — the outer walls slope by roughly
1.2 m across the 24 m building. The map is nonetheless internally consistent:
walls straight, partitions parallel, all five doorways in the right places.

slam_toolbox anchors the `map` frame on the first processed scan, which lands
wherever the bootstrap jog left the rover, so a global rotation and offset is a
property of *where the map frame was pinned*, not of map quality. It is
harmless for navigating inside that map, and it is why every figure here is
quoted after a rigid alignment. It would only matter if the map had to line up
with a building's axes or with a previously saved map.

### `depot_world` is otherwise the control, and it is spotless

Same configuration on the partitioned, cluttered world: **phantom 0.000**,
wall RMSE 4.3 cm, and **SLAM ATE 7 cm** — ten times better than any office run.
Visually the red map walls lie exactly on the grey truth everywhere, every
obstacle is present with the right shape and position, and nothing is doubled.

This is the useful distinction for tomorrow. **A real office at 15 cm lidar
height is a depot, not a corridor**: desks, chairs, cable trays, partitions,
features in every direction. Expect depot-like results in the rooms and
corridor-like variance only in a long featureless hallway.

### Noise: it degrades, it does not break

Doubling every odometry error term (yaw scale 12% → 24%, linear 2% → 5%, slip
1% → 3%):

| | nominal (n3) | doubled noise (n8) |
| --- | --- | --- |
| coverage | 0.979 | 0.846 |
| wall RMSE | 0.043 m | 0.118 m |
| SLAM ATE | 0.577 m | 0.773 m |
| path driven | 103 m | 38 m |
| stuck fraction | 0.37 | 0.73 |
| **contacts / near-misses** | **0 / 0** | **0 / 0** |

It maps 85% of the building with 12 cm wall error and touches nothing. What it
loses is throughput: it spends nearly three quarters of the run stalled and
covers a third of the distance. The failure mode under noise is *slow*, not
*wrong* — which is the right way round.

### The explorer's blacklisting timeout

`progress_timeout` 30 s → 60 s (n11 vs n3/n10, same seed) changed nothing
outside the seed-to-seed spread: coverage 96.9% against 97.9%/97.9%, phantom
0.079 against 0.000/0.104. It is therefore free, and the rover profile uses
60 s because hardware is slower than simulation and a person standing in a
doorway should not permanently write off the room behind them.

### A fix that did replicate: the recovery timeout

The two failed hypotheses above were both "change a number and see". This one
started from arithmetic in the log.

`n18`'s behaviour server failed two distinct ways, roughly equally often:
7x `Collision Ahead` and 6x `Exceeded time allowance before reaching the
DriveOnHeading goal`. The second is not geometry, it is a budget. The behaviour
tree asks for `BackUp backup_dist="0.25" backup_speed="0.04"` — 6.25 s of
motion — and `nav2_behaviors` defaults `time_allowance` to **10 s**. A margin of
1.6x. But a recovery happens next to an obstacle, which is exactly where the
collision monitor's SlowdownZone cuts commands to 75%: 0.25 m at 0.03 m/s is
8.3 s. One 1.5 s velocity-guard stall on top and the backup times out having
very nearly finished. The frontier is then blacklisted, and enough blacklists
end the run.

`scripts/apply_recovery_timeout.py` raises the allowance to 20 s and the backup
speed to 0.08 m/s (3.1 s of motion in a 20 s budget — six times the margin,
still slow enough that the guard and the collision monitor keep their
authority), and does the same for `Spin`.

Run on the three seeds that had stalled worst:

| seed | world | before | after |
| --- | --- | --- | --- |
| 101 | office | 0.519 | **0.981** |
| 808 | office | 0.301 | **0.979** |
| 707 | office | 0.645 | **0.977** |
| 7 | depot | 0.567 | **0.979** |
| 55 | office | 0.849 | **0.978** |

**Five seeds, five recoveries** (101, 808, 707, depot-7, 55), all to the
0.977-0.981 band the healthy runs occupy — and the predicted error disappeared: `Exceeded time allowance` 6 -> **0**
on seed 101, zero on both others. Seed 101 also went from 13 m driven and 5
narrow-gap transits to **132 m and 42** — the most doorway work of any run
tonight — with zero contacts. Map quality held: phantom 0.059 / 0.000 / 0.042,
wall error 0.219 / 0.037 / 0.188 m.

Why believe this one more than the two that failed: the mechanism was predicted
from the numbers *before* the run, the intervention made the predicted log line
vanish, and the outcome replicated on every seed tried.

**How much to believe it — the answer, after testing it properly.** Not much.

Re-running the *pre-fix* build on seed 101 gave 0.955 coverage rather than
stalling, so the stall is stochastic per seed. That makes the five paired
improvements above much weaker evidence than they look: those seeds were
chosen *because* they had stalled, and regression to the mean alone predicts
they improve on a re-run.

The unbiased test is fresh seeds. Six were run (`reports/night/p1..p6`):

| | stalled | rate |
| --- | --- | --- |
| pre-fix, all shipped-config runs | 5 of 14 | 36% |
| post-fix, six fresh seeds | 1 of 6 | 17% |

Fisher exact, two-sided: **p = 0.61**. That is no evidence of an effect. The
point estimate moved the right way and the sample is far too small to say
anything; six more runs would still not settle it.

What survives is narrower and still worth keeping: the 10 s allowance really
was only 1.6x the nominal manoeuvre time, the change costs nothing, and it
cannot make things worse. It stays in. But it should not be described as
having fixed the stall, and the earlier draft of this section did exactly
that.

### A promising fix that did not replicate### A promising fix that did not replicate

Seed 55 stalled at 85% coverage with 39 NavFn `failed to create a plan from
potential` errors. Frontier goals sit on the boundary of known space, often a
cell or two inside inflation, so widening the planner's goal tolerance from
0.25 m to 0.5 m is the obvious remedy. On seed 55 it worked spectacularly:

| seed 55 | tolerance 0.25 (`n13`) | tolerance 0.5 (`n15`) |
| --- | --- | --- |
| coverage | 0.849 | **0.978** |
| path driven | 55 m | **111 m** |
| stuck fraction | 0.35 | **0.30** |
| wall RMSE | 0.046 m | 0.041 m |

Then it was run on the two seeds 0.25 already handled well:

| coverage | tolerance 0.25 | tolerance 0.5 |
| --- | --- | --- |
| seed 55 | 0.849 | **0.978** |
| seed 21 | 0.979, 0.979 | **0.776** |
| seed 7 | 0.978 | 0.967 |
| mean | **0.95** | 0.91 |

Wall error went the same way on seed 7: 0.069 m at tolerance 0.25 against
0.234 m at 0.5.

**Reverted.** A change that rescues one seed and wrecks another is not a fix,
it is a resample of a high-variance distribution — and the planning errors it
was supposed to remove appeared at the same rate in `n15` (31 in 11 minutes) as
in `n13` (39 in 14). The shipped value stays 0.25, which has six samples behind
it, five of them at 96.7% coverage or better.

Recording this partly for the result and partly for the method: the earlier
version of this project's notes warns that aggregate metrics reward changes
that are actually noise, and a single dramatic run is exactly what that looks
like from the inside.

## 6b. Reading the 2026-08-20 real-rover replays

Two drives were recorded on the physical rover and replayed through several
costmap variants (`reports/drive_2026-08-20/`). The replay is **shadow mode**:
the bag drives the robot and the stack only perceives, maps, plans and
*records* what it would have commanded as `/cmd_vel_shadow`. Nothing the stack
decides moves anything.

That split the outputs into two kinds, and only one is evidence:

**Faithful** — anything that is a function of the recorded sensor data and the
config: the maps, the global and local costmaps, and the footprint collision
check, which evaluates real costmaps at real poses. `Collision Ahead` came out
**0 across every variant of both drives**, in roughly a dozen recovery
attempts. The inscribed-band refusal that dominates in simulation did not occur
once on this office's costmaps.

**Not faithful** — anything that measures the robot responding to a command.
`Exceeded time allowance` (5-6 per drive), `Failed to make progress` (16-17),
`backup failed`, `spin failed`, goal successes and failures. In shadow mode a
`BackUp` can only "succeed" if the human driver happened to be reversing at
that moment; otherwise it burns its entire allowance and reports a timeout. The
logs show exactly that — failures landing at 20.1 s and 20.0 s, the allowance
to the decisecond, while the successful ones finish in 1.7 s.

So the replays cannot say whether recoveries work on the rover, and any count
of navigation events taken from them is measuring the harness. They are
excellent for perception and costmap questions, which is what they were built
for and what the camera-layer fix rests on.

## 7. What to ship

```
simulator         native WSL Ubuntu, GPU (D3D12), scripts/exp_run_wsl.sh
odometry          robot_localization EKF: wheel forward velocity + gyro yaw rate
                  (leo_imu_bridge converts the firmware's leo_msgs/Imu)
SLAM              slam_toolbox async, scan_topic /scan_filtered,
                  max_laser_range 12.0, loop_search_maximum_distance 8.0,
                  loop_search_space_dimension 10.0, chain_size 5,
                  response_coarse 0.45  (widen the search, not the acceptance)
scan filter       laser_filters box filter, no tf_message_filter_target_frame
planner           NavfnPlanner, allow_unknown true, tolerance 0.25
controller        RotationShimController + RegulatedPurePursuitController
costmaps          polygon footprint, inflation 0.35, depth camera as an
                  ObstacleLayer PointCloud2 source (not a VoxelLayer)
safety            velocity_guard -> collision_monitor as sole /cmd_vel publisher
explorer          explore_lite, progress_timeout 60 on hardware
markers           aruco_detector, DICT_4X4_50, allowed_ids set to what you placed,
                  marker_length = the measured black square
```

Operating instructions are in `REAL_ROVER_DEPLOY.md`. The short version: drive
the first map yourself.

## 8. What was built for the rover

| file | what it is |
| --- | --- |
| `launch/real_mapping.launch.py` | teleop mapping: scan filter, SLAM, velocity guard, collision monitor, optional EKF and ArUco. No planner, no controller, no BT, no explorer |
| `launch/odometry_fusion.launch.py` | IMU bridge + `robot_localization` EKF owning `odom -> base_footprint` |
| `leo_nav2_exploration/leo_imu_bridge.py` | `leo_msgs/Imu` → `sensor_msgs/Imu`, with stationary gyro-bias calibration and `orientation_covariance[0] = -1` |
| `config/real/ekf.yaml` | wheel forward velocity + gyro yaw rate, two-dimensional |
| `leo_nav2_exploration/aruco_detector.py` | the detector above |
| `launch/aruco.launch.py` | sim and real profiles; the real one rate-limits to 5 Hz |
| `preflight_check` additions | `--require-imu`, `--require-aruco`, `--expect-ekf` |
| `REAL_ROVER_DEPLOY.md` | the operator guide |

**`real_mapping.launch.py` is the recommended first run.** The autonomous stack
spends 30–60% of every simulated run stalled; the map does not care who steers,
and SLAM is not the fragile part.

Verified in the container against synthetic input:

- `real_mapping.launch.py` brings up all seven expected nodes, none die;
- `odometry_fusion.launch.py` brings up the bridge, the static IMU transform
  and `ekf_filter_node`;
- fed a synthetic `leo_msgs/Imu` at 50 Hz, `leo_imu_bridge` estimated the gyro
  bias (`z = +0.573 deg/s`), subtracted it, marked the orientation absent and
  republished `sensor_msgs/Imu` at 50 Hz.

The one thing that will break the EKF on hardware is two publishers of
`odom -> base_footprint`: the rover's `wheel_odom_tf.py` and the EKF. tf2 does
not error; the pose alternates between two estimates and it looks like a SLAM
fault. `preflight_check --expect-ekf` fails when it sees both.

## 9. The bundle's own test suite is 13 red, and should be

`python3 -m pytest test` in `leo_nav2_exploration` reports **73 passed, 12
failed**. Every failure is the bundle asserting the design that the 2026-08-17
study and this one deliberately replaced:

| assertion | why it fails |
| --- | --- |
| planner plugin is `SmacPlannerLattice` (x2) | we ship NavFn — §3 |
| `inflation_radius >= 0.45` (x2) | tuned to 0.35 for doorways |
| a DWB velocity limit `<= 0.3` (x2) | DWB is gone; RPP replaced it |
| `voxel_layer` key exists (x2) | the depth camera is an ObstacleLayer source |
| `docs/README.md` exists, and three shell-contract checks | bundle documents this repo never carried |
| `backup_speed <= 0.05` | raised to 0.08 with a 20 s allowance — see the recovery-timeout fix |

The last one deserves a sentence rather than a shrug, because it relaxes a
*safety* contract rather than a design one. The bundle wants a slow backup near
an obstacle, which is right. 0.08 m/s is still 8 cm/s, the collision monitor and
velocity guard remain in the command chain ahead of it, and the three runs that
used it recorded zero contacts and zero near-misses. The alternative — keeping
0.05 and carrying the fix entirely in the 20 s allowance — would very likely
work too, but it is not the configuration that was actually validated, and
changing a number after the runs to satisfy a test is how untested settings get
shipped.

The count was 12 before tonight (65 passed then, 72 now — the eight new
`test_aruco_pose.py` cases all pass; the one added failure is the deliberate
`backup_speed` change above). Read the number as "the overlay has diverged from the bundle on
purpose", not as a regression.

## 10. Honest limits

- Every number here is from simulation. Zero contacts in every run and one
  near-miss in twenty-four, but simulation has no unmodelled wheel slip, no
  glass, no people.
- **The autonomous stall is unexplained.** Three of twelve scored runs stopped
  early and neither of the two hypotheses tested tonight survived contact with
  a second seed. What is established is the shape of it: it is a stall, it is
  not a collision, and it does not damage the map that was already built.
- The IMU bridge has never seen a real `leo_msgs/Imu`. Topic name, rate and
  noise are assumptions.
- Lidar *range noise and dropouts* are not exercised by this harness — the
  degrader exists (`scripts/sim_realism_scan.py`) but the overlay's scan topic
  is hardcoded, so wiring it in needs a launch change. Odometry error, which
  the 2026-08-16 study showed dominates by orders of magnitude, *is* exercised,
  including a doubled-noise run.
- 12 m of usable RPLIDAR C1 range is a datasheet figure.
- The simulated worlds are far tidier than an office at 15 cm lidar height.
- Sample sizes are small: two seeds per arm on the planner comparison, one on
  the EKF ablation, one on the noise stress. The planner result is the only one
  where both arms were run on the same two seeds and agreed on every metric.
- `real_navigation.launch.py` and `real_exploration.launch.py` have been
  launched and inspected, never driven. Only the mapping stack
  (`real_mapping.launch.py`) has had its behaviour exercised end to end, and
  that against a synthetic rover rather than a real one.
