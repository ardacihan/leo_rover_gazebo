# Leo Rover Gazebo Simulation

## Requirements

* Ubuntu 22.04 (Jammy)
* ROS 2 Humble
* Gazebo Harmonic (`gz-sim8`)
* Leo rover model https://github.com/LeoRover/leo_common-ros2 
* Custom office map https://github.com/husarion/husarion_gz_worlds.git" 

---

# Docker Setup

## 1. Build the Docker Image

```bash
docker build -t leo_rover_humble .
```

## 2. Allow X11 Forwarding

Run on the host machine:

```bash
xhost +local:docker
```

## 3. Start the Container

```bash
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/ros2_ws \
  leo_rover_humble
```
Make sure to enable docker to use gpu with nvidia drivers. To use nvidia GPU in container run:

```bash
xhost +local:docker

docker run -it --rm \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/ros2_ws \
  leo_rover_humble
```

then in container 
```bash
apt update && apt install -y mesa-utils
```

Get in existing container from other terminals

```bash
docker exec -t -i <container name> /bin/bash
```

## 4. Build the Workspace

Inside the container:

```bash
cd /ros2_ws/src
git clone https://github.com/husarion/husarion_gz_worlds.git
```
```bash
cd /ros2_ws
colcon build --symlink-install 
source install/setup.bash
clear
```
---

# Launch Gazebo (Select world; aws_room, warehouse or husarion_office)

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py world:=husarion_office
```

# Keyboard Controls

Open a new terminal inside the container:

```bash
cd /ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/leo1/cmd_vel
```

# Launch RViz2

```bash
rviz2 --ros-args -p use_sim_time:=true
```

# Run SLAM Toolbox

Open another terminal:

```bash 
ros2 launch leo_rover_gazebo slam.launch.py
```


# Launch Nav2

```bash
 ros2 launch leo_rover_gazebo nav2.launch.py
``` 

---

## Multi-Robot Shared Mapping Demo

This demo starts the full two-robot setup:

```text
Gazebo + leo1/leo2 + SLAM + alignment manager + shared map merger
```

Package: `multi_robot_shared_mapping`

### Data flow (current version)

```text
/leo1/scan + /leo1/odom  ->  /leo1/map
/leo2/scan + /leo2/odom  ->  /leo2/map

Optional AprilTags:
  /leoN/camera/image  ->  apriltag_detection_node  ->  /leoN/tag_detections
  persistent landmarks ->  /leoN/apriltag_landmarks, /shared/apriltag_landmarks

Alignment manager (map_based_aligner):
  /leo1/map + /leo2/map [+ tag hint]  ->  candidate transform
  accepted transform only  ->  /map_based_transform/leo2_to_leo1
  candidate always         ->  /alignment_candidate_transform
  diagnostics              ->  /alignment_confidence, /alignment_debug_json

Shared map merger:
  accepted transform only  ->  /shared_map
  preview                  ->  /shared_map_candidate
  debug                    ->  /shared_map_raw, /shared_map_cleaned
```

Ground truth spawn offsets are **never** used to build `/shared_map`. They may be enabled only for optional evaluation logs (`enable_alignment_evaluation:=true`).

### Launch modes

| `alignment_mode` | Behavior |
|------------------|----------|
| `fixed` (default) | Static `leo2/map -> leo1/map` offset from launch args |
| `map` | Occupancy-grid matching only |
| `tag` | Tag hint + map validation when maps exist |
| `hybrid` | Tag hint refines map search window; map overlap validates |

### Recommended launch (collaborative exploration)

```bash
ros2 launch multi_robot_shared_mapping shared_mapping_demo.launch.py \
  enable_apriltag_detection:=true \
  enable_tag_alignment:=true \
  enable_map_alignment:=true \
  alignment_mode:=hybrid
```

Fixed-offset baseline (debug / comparison):

```bash
ros2 launch multi_robot_shared_mapping shared_mapping_demo.launch.py \
  alignment_mode:=fixed \
  robot2_to_shared_x:=2.36 \
  robot2_to_shared_y:=-11.27 \
  robot2_to_shared_yaw:=0.0
```

### Key launch arguments

| Argument | Default | Purpose |
|----------|---------|---------|
| `enable_apriltag_detection` | `false` | Spawn tags + run detection node |
| `enable_tag_alignment` | `false` | Run `tag_based_map_aligner` |
| `enable_map_alignment` | `false` | Run `map_based_aligner` (also starts if tag alignment enabled) |
| `alignment_mode` | `fixed` | `fixed \| map \| tag \| hybrid` |
| `min_alignment_confidence` | `0.5` | Base acceptance threshold |
| `landmark_persistence` | `true` | Tags never forgotten once seen |

### Key topics to monitor

```bash
ros2 topic echo /alignment_confidence
ros2 topic echo /alignment_debug_json
ros2 topic echo /alignment_recovery_goal
ros2 topic echo /map_based_transform/leo2_to_leo1
ros2 topic echo /shared_map_candidate
```

`/alignment_debug_json` includes: `confidence_level`, `exploration_allowed`, `common_landmark_count`, `map_overlap_score`, `ambiguity_score`, `recommended_action`, accepted/candidate transforms.

### AprilTags

Fixed AprilTags (tag36h11) are spawned from `config/apriltag_landmarks.yaml` when `enable_apriltag_detection:=true`. Positions match inline Gazebo SDF planes.

- `tag_based_map_aligner` publishes `/estimated_transform/leo2_to_leo1` (hint, not directly fused)
- One common tag = weak/medium anchor; **not** required before exploration
- Two/three+ common tags increase confidence; map matching still validates

### Unit tests (no Gazebo)

```bash
cd /ros2_ws
PYTHONPATH=src/multi_robot_shared_mapping python3 -m pytest src/multi_robot_shared_mapping/tests
```

### Save outputs

```bash
ros2 run multi_robot_shared_mapping save_shared_outputs
```

Writes to `maps/`:
- `shared_office_map.pgm` + `.yaml`
- `apriltag_landmarks_merged.yaml`
- `alignment_debug.json`

Or save the occupancy grid directly:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/shared_office_map --ros-args -r map:=/shared_map
```

If SLAM maps are corrupted (robot hits a wall), alignment and merging become unreliable. Drive slowly.

Optional: patch the Husarion world with embedded tags:

```bash
python3 src/multi_robot_shared_mapping/scripts/patch_husarion_office_apriltag_world.py
```

---

## Map Matching — Technical Report

This section documents exactly how occupancy-grid map matching works in the current pipeline.

### 1. Purpose

Estimate the 2D rigid transform **leo2/map → leo1/map**:

```text
[x_leo1, y_leo1]^T = R(yaw) @ [x_leo2, y_leo2]^T + [dx, dy]^T
```

Map matching runs **before any common AprilTags exist**. Tags refine or validate the estimate but are not mandatory for collaborative exploration.

Implementation: pure NumPy module `grid_map_matching.py`, orchestrated by ROS node `map_based_aligner.py`.

### 2. Input extraction

Every 5 s (`match_period_sec`), the aligner reads `/leo1/map` and `/leo2/map`.

From each occupancy grid:

1. **Occupied points** — cells with value ≥ `occupied_threshold` (default 50)
2. **Free points** (leo1 only) — cells with `0 ≤ value < occupied_threshold`

Both are converted to world coordinates `(x, y)` in each robot's map frame, then downsampled:

- Voxel size: `match_resolution` (default **0.15 m**)
- Max points: **400** occupied points per map
- Requires ≥ **100** occupied cells per map or matching is skipped

Free-space points from leo1 (up to 1600) are used only for conflict penalization, not for positive scoring.

### 3. Target lookup grid (leo1)

leo1 occupied points are rasterized into a weighted 2D lookup grid at `match_resolution`:

| Cell content | Weight |
|--------------|--------|
| Exact occupied | +2 |
| Dilated neighbor (1 cell) | +1 |
| Known free (leo1) | −1 |
| Unknown | 0 |

Free-space weights never overwrite occupied/dilated cells. This means transformed leo2 walls landing in leo1 **known free space** actively **lower** the score instead of being ignored.

### 4. Search strategy

**Coarse-to-fine exhaustive search** over `(dx, dy, yaw)`.

#### Search window (depends on mode)

| Condition | Center | XY range | Yaw range |
|-----------|--------|----------|-----------|
| Tag/hybrid hint available | Tag estimate | ±2.0 m | ±0.35 rad (~20°) |
| Previously accepted transform | Accepted `(dx,dy,yaw)` | ±2.0 m | ±0.35 rad |
| No tags, no prior accept | `(0, 0, 0)` | ±15.0 m | ±π rad |

Hybrid mode no longer blocks when no tag exists — it falls back to full map search.

#### Coarse pass

- XY step: **0.75 m**
- Yaw step: **15°**
- For each yaw: rotate all leo2 points, score every `(dx, dy)` translation by summing lookup weights at transformed point locations
- Tie-break: prefer candidate closest to search center (tiny penalty on distance and yaw delta)
- Keep top **3 translations per yaw** for ambiguity analysis

#### Fine passes (2 iterations)

- Window shrinks to previous step size
- XY step: `max(prev/5, 0.05 m)`
- Yaw step: `max(prev/5, 1°)`

### 5. Scoring metrics

For the best candidate:

| Metric | Formula / meaning |
|--------|-------------------|
| `overlap_score` | Count of leo2 points landing on leo2 weight > 0 |
| `normalized_overlap_score` | `overlap_score / num_source_points` ∈ [0, 1] |
| `free_space_conflict_ratio` | Fraction of leo2 points landing on leo1 known-free cells |

**Positive signal:** wall-on-wall overlap (leo2 occupied → leo1 occupied/dilated).

**Negative signal:** leo2 occupied → leo1 known free (physically implausible alignment).

### 6. Ambiguity detection

From coarse top-K candidates (`select_top_candidates`, k=5, NMS separation ≥0.5 m / 10°):

- Sort by score descending
- Find first candidate that is **genuinely different** from best (≥1.0 m or ≥15° away)
- `ambiguity_ratio = second_best_score / best_score`
- **Ambiguous** if ratio ≥ **0.85** (symmetric corridors, repeated wall patterns)

Ambiguous map-only matches are **never accepted** — they remain on `/alignment_candidate_transform` and `/shared_map_candidate` only.

### 7. Preflight rejection gates

Before confidence scoring, a candidate is rejected if:

| Gate | Default threshold |
|------|-------------------|
| Match failed | No occupied points |
| Low overlap | `normalized_overlap_score < 0.25` |
| Free-space conflict | `free_space_conflict_ratio > 0.15` |
| Ambiguous (0 common tags) | Candidate only, not accepted |
| Tag-map disagree (1 tag, hybrid/tag) | Translation > **1.0 m** or yaw > **15°** |
| Severe local map quality | Either robot quality tracker `is_severe` |

### 8. Final confidence composition

Weighted mean over available components (`alignment_confidence.py`), renormalized when optional inputs are missing:

| Component | Weight | Source |
|-----------|--------|--------|
| Occupancy overlap | 0.30 | `normalized_overlap_score` |
| Free-space conflict | 0.15 | `1 - conflict_ratio/threshold` |
| Unambiguity | 0.15 | `1 - ambiguity_ratio` |
| Transform stability | 0.10 | Proximity to last accepted transform |
| Local map quality | 0.10 | `min(leo1, leo2)` quality score |
| Tag alignment | 0.10 | From tag aligner (hybrid/tag modes) |
| Tag residual | 0.05 | `1 / (1 + mean_residual)` |
| Landmark count | 0.025 | `min(1, count/4)` |
| Landmark spread | 0.025 | `min(1, spread/2m)` |

### 9. Confidence levels and acceptance policy

`exploration_policy.py` maps numeric confidence + context to **low / medium / high**:

| Situation | Typical level | Acceptance floor |
|-----------|---------------|------------------|
| 0 tags, strong overlap (≥0.45), unambiguous | medium | ~0.42 |
| 1 tag + map agree + overlap ≥0.25 | medium | ~0.38 |
| 1 tag alone (no map agreement) | low–medium | ≥0.50 (harder) |
| 2+ tags, good spread | medium–high | ~0.42 |
| 3+ tags, low residuals | high | ~0.40 |
| Ambiguous | low | 1.0 (never accept) |

**Accepted-vs-candidate state machine** (`alignment_state.py`):

- Every new estimate is a **candidate** first
- Promoted to **accepted** only if:
  - Confidence ≥ dynamic floor
  - Transform jump ≤ **2.0 m**, yaw jump ≤ **25°** (vs last accepted)
  - Confidence improves prior accepted by ≥ **0.05** (when consistency required)
- Rejection keeps previous accepted transform and stable `/shared_map`

Only the **accepted** transform is published on `/map_based_transform/leo2_to_leo1`.

### 10. Shared map fusion gating

`shared_map_merger.py` fuses leo2 into `/shared_map` only when:

- Accepted transform is valid
- `confidence_level` is **medium** or **high**, OR raw confidence ≥ `min_alignment_confidence`
- Local map quality ≥ **0.35** for both robots

Otherwise:
- Republishes last accepted grid if available
- Falls back to leo1-only map
- Always publishes `/shared_map_candidate` from latest candidate transform

### 11. Collaborative exploration behavior

Robots are **not** required to collect 2+ common tags before exploring separately.

| `exploration_allowed` | Meaning |
|-----------------------|---------|
| `true` | Robots should take separate frontiers (`recommended_action: explore_separate_frontiers`) |
| `false` | Hold or occasional recovery (ambiguous match, poor map quality, low confidence) |

Recovery (`/alignment_recovery_goal`) is **optional** — e.g. scan a nearby tag or overlap region — not a prerequisite for normal exploration.

### 12. Debug outputs

| Topic | Content |
|-------|---------|
| `/leo2/map_transformed_debug` | leo2 map warped into leo1 frame (best candidate) |
| `/shared_map_candidate` | Merged map using candidate transform |
| `/alignment_debug_json` | Full diagnostic JSON |
| `/alignment_recovery_goal` | Recovery recommendation JSON when needed |

### 13. Source files

```text
grid_map_matching.py      Pure matching math
map_based_aligner.py      ROS alignment manager node
alignment_confidence.py   Multi-component confidence + ambiguity
alignment_state.py        Accepted vs candidate state machine
exploration_policy.py     Confidence tiers + exploration/recovery policy
shared_map_merger.py      Accepted-only map fusion
recovery_advisor.py       Optional recovery recommendations
tests/test_grid_map_matching.py
tests/test_exploration_policy.py
```

---

## Multi-Robot Setup

### Terminal 1: Start Docker

From the project root on the host machine:

```bash
cd ~/Projects/leo_rover/leo_rover_gazebo
xhost +local:docker

docker run -it --rm \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/ros2_ws \
  --name leo_rover_dev \
  leo_rover_humble
```

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
```

---

### Build

Inside Docker:

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select multi_robot_shared_mapping leo_rover_gazebo
source install/setup.bash
```

---

## Run Mapping

### Terminal 1: Launch Gazebo, Robots, SLAM, and Shared Map

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch multi_robot_shared_mapping shared_mapping_demo.launch.py
```

---

## RViz

### Terminal 2: Open RViz

From a new host terminal:

```bash
docker exec -it leo_rover_dev /bin/bash
```

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

### RViz Setup

Set:

```text
Fixed Frame: leo1/map
```

Add displays:

```text
Add -> By topic -> /leo1/map              -> Map1
Add -> By topic -> /leo2/map              -> Map2
Add -> By topic -> /shared_map            -> SharedMap (accepted)
Add -> By topic -> /shared_map_candidate  -> SharedMapCandidate
Add -> By topic -> /leo2/map_transformed_debug -> Leo2AlignedDebug
Add -> By topic -> /shared/apriltag_landmarks  -> MarkerArray
Add -> By topic -> /leo1/scan             -> LaserScan1
Add -> By topic -> /leo2/scan             -> LaserScan2
Add -> By display type -> TF
```

Optional: rename the map displays in RViz.

Note: `/shared_map` is the accepted stable merge. `/shared_map_candidate` shows the latest alignment attempt before acceptance. RViz Fixed Frame should be `leo1/map`.


---

## Drive Robots

### Terminal 3: Drive leo1

From a new host terminal:

```bash
docker exec -it leo_rover_dev /bin/bash
```

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/leo1/cmd_vel
```

---

### Terminal 4: Drive leo2

From a new host terminal:

```bash
docker exec -it leo_rover_dev /bin/bash
```

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/leo2/cmd_vel
```

Use normal `teleop_twist_keyboard` controls.

---

## Important SLAM Note

Drive slowly.

If a robot pushes into a wall while odometry still changes, SLAM can create a shifted or smeared map.


---

## Save Shared Map

Inside Docker:

```bash
cd /ros2_ws
source install/setup.bash
mkdir -p /ros2_ws/maps

# Save map + landmarks + alignment debug JSON
ros2 run multi_robot_shared_mapping save_shared_outputs

# Or occupancy grid only via nav2 map_saver
ros2 run nav2_map_server map_saver_cli \
  -f /ros2_ws/maps/shared_office_map \
  --ros-args -r map:=/shared_map
```