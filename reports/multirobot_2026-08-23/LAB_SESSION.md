# Two rovers in the lab — what to print, what to type, what to check

Written 2026-08-24 from the simulation work in `REPORT.md`. **This is not yet a
finished deployment guide.** Three of the four real-stack launches now take a
`robot_ns`; `real_mapping.launch.py` does not, and nothing here has been run on
hardware. What *is* settled — the marker card, the DDS plan, the coordination
decision, and the failure signatures worth recognising — is written down so the
next session starts from decisions rather than from questions. Section 4 says
exactly what is done and what is not.

Read `REPORT.md` first for what was actually measured.

---

## 1. The marker card — print this and tape it inside the lid

```
+---------------------------------------------------------------+
|  ArUco markers for two-rover mapping                          |
|                                                               |
|  Dictionary  DICT_4X4_50        (NOT 4X4_1000, NOT 6X6)       |
|  IDs         1 - 8              (0 and 9 exist, unused)       |
|  Black square side   200 mm     <- this is `marker_length`    |
|  White border        >= 34 mm all round (one cell)            |
|  Sheet size          268 x 268 mm minimum                     |
|  Mounting height     300 mm to the CENTRE of the square       |
|  Mounting            flat on the wall, facing into the room   |
|                                                               |
|  Detector parameter:   marker_length := 0.20                  |
|  Detector parameter:   frame_is_optical := true  (RealSense)  |
+---------------------------------------------------------------+
```

**`marker_length` is the side of the black square, including its black border
cell — not the paper, not the white margin.** It is the one number a deployment
can get wrong with nothing erroring: the pose simply lands short or long along
the view ray, proportionally. At 0.15 instead of 0.20 every marker sits 25% too
near.

Three things to reconcile before printing:

- **`tools/make_aruco_print_pdf.py` currently prints 80 mm markers from
  `DICT_4X4_1000`, ids 0–9.** Both differ from what the detector expects
  (`DICT_4X4_50`, and 80 mm is far too small for the ranges below). Fix the tool
  or set the detector to match — but pick one and put it on the card.
- **`allowed_ids`** must list exactly the ids taped up. It defaults to
  `[1..8]`. Anything else is rejected outright, which is deliberate: a sim run
  produced a spurious id 25 from ordinary office rectangles.
- **Measure one printed marker with a ruler before trusting the PDF.** Printer
  scaling is the classic silent 4% error.

### Range and placement, from the sim measurements

| range | marker in image | verdict |
|---|---|---|
| 1.5 m | ~90 px | 1.5 cm position error — excellent |
| 2.1 m | ~55 px | 8 cm error — good |
| 3.0 m | ~40 px | usable |
| 4.5 m | ~28 px | `max_range` cap — noisy |
| 5.5 m | ~25 px | **map position wandered metres while the rover turned** |

`max_range` is set to **4.5 m**. Beyond that a detection is worse than none,
because landmarks are persistent: one bad placement anchors the outlier gate and
poisons later good sightings.

**Incidence angle matters as much as range.** A marker seen 59° off its normal
is only 0.10 m wide in the image (~14 px) and falls under the 18 px floor; at
82° it is invisible. In sim this cost a rover *every* detection for 300 frames.
So: **mount markers facing into the space the rovers will drive**, not along a
wall.

**Both rovers must see the SAME markers** — that is the entire mechanism. Put at
least **three** markers in the corridor or shared area both rovers must cross,
facing that space. Two is the minimum for a transform; three gives the fit
something to disagree with.

**And do not put them in a line.** This is the one placement rule that cost a
run. On the depot capture run the four markers both rovers saw spanned 13.8 m
north–south but only 3.0 m east–west — very nearly collinear. Collinear
landmarks pin down translation *along* the line and leave the **rotation**
poorly conditioned, so one mediocre landmark at the end of the line levers the
whole yaw. The published transform came out 2.25 m and 14.8° wrong from
landmarks that were individually accurate to 0.2 m, and a plain fit over the
same four gives 0.37 m / 0.6°.

Rule of thumb: the shared markers should enclose area, not describe a line —
spread them across **two roughly perpendicular walls**, so the set is at least
half as wide as it is long. Check it before taping anything down: sketch the
positions and confirm they form a triangle you would be happy to measure an
angle from.

---

## 2. The DDS plan

Constraint first: there is a recorded finding on this branch (commit `d241087`)
that **our own DDS traffic starves the rover's firmware**. So the plan is not
"put everything on domain 0" — it is to keep the heavy topics on the machine
that produces them.

**Domains.** One shared domain for the fleet, `ROS_DOMAIN_ID=42`, on every rover
and the laptop. Different domains would isolate the rovers completely and there
would be no shared map at all. Isolation is done by topic scope, not by domain.

**Do not rely on default multicast.** Use a CycloneDDS peer list so discovery is
deterministic on a lab LAN that may have other ROS machines on it:

```xml
<!-- /etc/cyclonedds/fleet.xml on every machine -->
<CycloneDDS><Domain Id="42">
  <General><AllowMulticast>false</AllowMulticast></General>
  <Discovery><Peers>
    <Peer Address="192.168.1.11"/>  <!-- rob_a  -->
    <Peer Address="192.168.1.12"/>  <!-- rob_b  -->
    <Peer Address="192.168.1.10"/>  <!-- laptop -->
  </Peers></Discovery>
</Domain></CycloneDDS>
```

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/fleet.xml
```

**What crosses the network, and what must not.**

| topic | rate / size | crosses? |
|---|---|---|
| `/{ns}/map` | ~0.2 Hz, 100 kB–1 MB | **yes** — the merge needs it |
| `/{ns}/tag_detections` | on sighting, tiny | **yes** — this is the rendezvous |
| `/{ns}/exploration_claims` | 1 Hz, tiny | **yes** |
| `/shared_map` | ~0.2 Hz | **yes**, laptop → rovers |
| `/tf`, `/tf_static` | 50 Hz, small | **yes** — needed for peer poses |
| `/{ns}/scan` | 10 Hz, ~30 kB | **no — rover-local** |
| `/{ns}/camera/**` (image, points) | 5–30 Hz, MB | **no — rover-local.** This is the one that starves the firmware |
| `/{ns}/**/costmap` | 1–5 Hz, large | **no — rover-local** |
| `/{ns}/odom`, `/{ns}/imu/data` | 30–100 Hz | **no — rover-local** |

The heavy topics are all *inputs* to something that runs on the same rover, so
keeping them local costs nothing. Enforce it rather than hoping: run the
per-rover stack (SLAM, Nav2, cloud filter, ArUco detector) **on the rover**, and
only the merger, the aligners and the operator tooling on the laptop.

Sanity check before trusting anything: on the laptop, `ros2 topic list` should
show `/rob_a/map` and `/rob_b/map` but **not** `/rob_a/scan` or any camera
topic. If it does, the bandwidth plan is already broken.

---

## 3. Coordination: the decision, and why

**Chosen: physical spatial partitioning, not runtime coordination.**

`explore_lite` — what the real rover actually runs — has no coordination hook,
and giving it one was out of scope. Two coordination implementations exist in
the tree (`exploration_policy.py`, `coordination.py`); both are pure, unit
tested, and neither is wired to `explore_lite`.

The sim result argues the same way. On `depot_world`, coordinated exploration
produced a merged map of 136.5 m² against 132.7 m² for one rover alone — the
second rover added **3.8 m²** nothing else had mapped, and both trajectories
covered the whole world. Coordination cannot begin until the rovers have seen
common markers (peer poses are a TF lookup), and by then they have both been
nearly everywhere. **In a room-sized space the rendezvous arrives too late to
buy anything.**

So for the lab: **start the two rovers in different rooms and let them run
independently, and merge their maps.** Report coordinated allocation as a
sim-only result. The value on the day is the shared map and the recovered
transform, not the frontier allocation.

---

## 4. Bring-up order

**Partly runnable.** State of the namespacing, honestly:

| launch | `robot_ns`? | note |
|---|---|---|
| `navigation_overlay.launch.py` | **done** | plus `cloud_input_topic`; the hardcoded `/rob_4` depth cloud is now derived from `robot_ns` |
| `real_navigation.launch.py` | **done** | threads `robot_ns` and `cloud_input_topic` into the overlay |
| `real_exploration.launch.py` | **done** | namespaces `explore_node` and prefixes `robot_base_frame`; `costmap_topic: map` is relative so the namespace moves it for free |
| `real_mapping.launch.py` | **done (2026-08-25 night)** | default `robot_ns` empty = byte-for-byte the 2026-08-20 field configuration; see below |

Defaults are unchanged, so single-rover behaviour is byte-for-byte what the
2026-08-20 field runs used. All four were verified to load with
`ros2 launch ... -s` (real_mapping in both the default and `robot_ns:=rob_a`
forms); **none has been run on hardware.**

`real_mapping.launch.py` namespacing (done overnight 2026-08-25, in sim only):
per-node `namespace=`, the load-bearing `('/map', '/{ns}/map')` and
`('/map_metadata', ...)` remaps on slam_toolbox, prefixed
`odom_frame`/`base_frame`/`map_frame`, the full cmd_vel chain
(`/{ns}/cmd_vel_nav → … → /{ns}/cmd_vel`) and scan chain
(`/{ns}/scan → /{ns}/scan_filtered`, slam on `/{ns}/scan_uniform`)
prefixed, `/tf`, `/tf_static` kept global on every node. Two guards worth
knowing: `use_ekf:=true` together with `robot_ns` refuses to launch (the EKF
belongs to `real_bringup.launch.py` in multi-rover mode — two EKFs fighting
over `odom -> base_footprint` is worse than an error), and the velocity
guard's `odom_topic` becomes `/{ns}/wheel_odom` — confirm the rover's driver
publishes there or the guard will hold the rover still.

Also flag: `real_mapping.launch.py` defaults `marker_length` to **0.15**. Use
the card's number.

**Before the rovers are switched on**

1. Tape the markers. At least three in the shared corridor facing into it,
   300 mm to centre. Note the ids on paper.
2. Measure one printed marker. Confirm 200 mm black square.
3. Pace out and write down each rover's start pose in a common frame — this is
   the ground truth the recovered transform gets scored against. Without it the
   session produces a number nobody can check.

**On each rover** (`rob_a`, then `rob_b`)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/fleet.xml

# 1. drivers + wheel odom + EKF   (per rover)
ros2 launch leo_rover_real_bringup real_bringup.launch.py robot_ns:=rob_a

# 2. mapping + navigation
ros2 launch leo_nav2_exploration navigation_overlay.launch.py \
    profile:=real_root robot_ns:=rob_a

# 3. ArUco detector  -- the numbers from the card
ros2 launch leo_nav2_exploration aruco.launch.py \
    profile:=real robot_ns:=rob_a \
    marker_length:=0.20 max_range:=4.5 min_hits:=3 \
    allowed_ids:=1,2,3,4,5,6,7,8 \
    detection_topic:=/rob_a/tag_detections \
    samples_file:=/tmp/aruco_samples_rob_a.csv
```

**On the laptop**

```bash
ros2 launch multi_robot_shared_mapping shared_align.launch.py \
    use_sim_time:=false alignment_mode:=hybrid \
    enable_tag_alignment:=true enable_map_alignment:=true \
    compare_to_ground_truth:=true \
    ground_truth_x:=<paced> ground_truth_y:=<paced> ground_truth_yaw:=<paced>
```

Then drive both rovers — teleop first, always — through the shared corridor so
each sees the markers.

**Marker-free fallback (new, night 2026-08-25).** If the markers fail (bad
print, wrong ids, lighting), the merge no longer depends on them:
`alignment_mode:=markerfree` runs the benchmarked global grid matcher with
margin abstention — 6/7 recorded pairs within 0.5 m/10°, zero
confident-wrong, and three live sim locks at 0.18–0.45 m including
depot-style symmetric rooms. It abstains (honestly, with the reason on
`/alignment_debug_json`) until the maps overlap enough; expect the lock
later than the tag path (~10 min vs 5–7). Markers remain the primary for
the demo; this is the "no babysitting" insurance.

**Distributed option (new, night 2026-08-25).** To remove the laptop as a
single point of failure, run the per-rover merger on each rover instead of
`shared_align` on the laptop:

```bash
ros2 launch multi_robot_shared_mapping distributed_shared_map.launch.py \
    use_sim_time:=false alignment_mode:=markerfree
```

Each rover then merges the peer's map locally and publishes its own
`/leo{i}/shared_map` (own frame — the explorer mask needs no alignment TF).
Verified in sim: both rovers locked independently, their merges agreed to
0.28 m/0.6°, and both shared maps stayed alive after the laptop node was
killed mid-run. The launch currently uses the leo1/leo2 names; for rob_a/
rob_b, edit the two `_rover_pair(...)` lines at the bottom of the launch
file — 30 seconds, flagged here so nobody hunts for a parameter that does
not exist yet.

**Clock synchronisation (check before trusting anything cross-rover).**
Peer poses are TF lookups across machines and skew turns straight into
position error — 1 s of skew at 0.26 m/s is 26 cm, comparable to the whole
alignment error budget. On each machine and the laptop:

```bash
timedatectl status        # NTP active: yes, and the same source
ntpdate -q <laptop-ip>    # offset should be << 0.1 s
```

If there is no NTP on the lab network, run chrony on the laptop and point
both rovers at it before starting anything. The sim could not rehearse
this (single clock); it is the one bring-up step with zero overnight
coverage — do it first.

---

## 5. What to check before believing anything

In this order. Each one has cost a whole run in sim.

1. **`ros2 topic info /rob_a/map -v`** — publisher count ≥ 1, and the same for
   `rob_b`. Zero means the per-robot remap regressed and both SLAMs are
   clobbering one `/map`.
2. **`ros2 topic info /rob_a/tag_detections`** — type must be
   `visualization_msgs/msg/MarkerArray`. A different ArUco node in this tree
   publishes a `String` and would satisfy a topic-name check while the aligner
   silently receives nothing.
3. **Stand a rover 1.5 m square-on to a marker.** The log must say
   `ArUco N CONFIRMED at map (x, y, z)`. Compare that to the tape measure. If it
   is short by a quarter, `marker_length` is wrong.
4. **`ros2 topic echo /alignment_locked`** — false until both rovers have seen
   common markers. That is correct, not a fault.
5. **When it locks, check the transform against your paced ground truth.**
   Inside 0.5 m and 10° is good. Outside it, do not trust the merged map.
6. **Look at the merged map.** Doubled or offset-parallel walls mean the
   alignment is wrong however confident it claims to be.

---

## 6. When it stalls

| symptom | cause | do |
|---|---|---|
| `/alignment_locked` never true | rovers have not seen the same markers | drive both through the shared corridor; check `/rob_*/tag_detections` is non-empty for each |
| Locked, but the merged map has doubled walls | alignment wrong despite confidence | check the tag-vs-grid disagreement in the bridge log; trust the tags |
| Merged map is a rotated copy of itself | grid matcher found a 90°/180° flip | expected in rectilinear rooms; the bridge should already be refusing it — check `require_tag_evidence` is true (hybrid mode). In `markerfree` mode the margin abstention is the defense: 0 flips in 3 live runs + 10 benchmark pairs, but if you ever see one, stop trusting the merge and fall back to markers |
| One rover's map is fragmented, walls dashed | that rover's SLAM is diverging | stop and restart it; check the EKF is actually running and the IMU topic is live |
| A rover sits still, camera sees a blank wall | wedged | it has no escape; move it by hand and note the spot |
| Confidence high, transform wrong | residuals measure self-consistency, not correctness | get a third common marker with good spread |

**The single most useful diagnostic**: save both per-rover maps and re-fuse them
offline under different transforms (`scripts/fuse_maps_offline.py`). It takes a
second and separates "the maps are bad" from "the transform is bad" — which are
completely different problems with completely different fixes.

---

## 7. Known-good numbers, for reference

From the sim runs in `REPORT.md`. Treat as expectations, not guarantees.

- Tag position error at 1.5 m: **1.5 cm**; at 2.1 m: **8 cm**.
- Recovered transform, good run: **0.12 m / 1.1°** over a 13.8 m separation.
- Recovered transform, small world: **0.23 m / 0.6°**.
- SLAM drift after ~19 m of driving: **~3 m** — this is what breaks large spaces.
- Time to first alignment lock: **5–7 minutes** of driving, once both rovers are
  moving through shared space.
- Marker-free lock (night 2026-08-25, n=3): **0.18–0.45 m / 0.0–1.3°**,
  arriving at ~10–11 minutes; 11–37 honest abstentions first is normal.
- Per-rover (distributed) merges agree with each other to **~0.28 m / 0.6°**
  (n=1).
