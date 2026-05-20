# Leo Rover Gazebo Simulation

## Requirements
* **OS:** Ubuntu 22.04 (Jammy)
* **ROS 2:** Humble
* **Gazebo:** Harmonic (gz-sim8)


## Docker Setup (Recommended)

1. **Build the Docker Image:**
```bash 
2. docker build -t leo_rover_humble .
```


2. **Allow X11 Forwarding (Run on your host machine):**

```bash
xhost +local:docker
```

3. **Run the Container:**
```bash
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/ros2_ws \
  leo_rover_humble

```


4. **Build the Workspace (Inside the container):**
```bash
cd /ros2_ws
colcon build --symlink-install --packages-skip leo_rover_slam
source install/setup.bash

```



---

## Running the Project

### 1. Launch the Gazebo Simulation

Spawns 3 rovers isolated by namespace (`/leo1`, `/leo2`, `/leo3`).

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py

```

### 2. Control a Rover via Teleop

Open a new terminal session in the container and drive a specific rover (e.g., `leo1`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/leo1

```

---

## Mapping & Navigation (Nav2 Stack)

### 1. Run SLAM Toolbox

Directly bind the SLAM mapping node to a targeted rover's transform frames and sensor topics:

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true \
  -p odom_frame:=leo1/odom \
  -p base_frame:=leo1/base_link \
  -p map_frame:=map \
  -r /scan:=/leo1/scan

```

*Drive the rover around using the teleop terminal to populate map grid data.*

### 2. Save the Generated Map

Once the map looks complete in RViz, save it to your workspace:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/my_gazebo_map --ros-args -p use_sim_time:=True

```

### 3. Run Autonomous Navigation (Nav2)

Launch the localization, costmap calculation, and path planning stack within the target rover's namespace:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  namespace:=leo1 \
  use_namespace:=True \
  map:=/ros2_ws/my_gazebo_map.yaml \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml

```