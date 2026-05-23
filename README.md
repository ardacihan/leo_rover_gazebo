# Leo Rover Gazebo Simulation

## Requirements

* Ubuntu 22.04 (Jammy)
* ROS 2 Humble
* Gazebo Harmonic (`gz-sim8`)

---

# Docker Setup

## 1. Build the Docker Image

```bash id="ny9bjj"
docker build -t leo_rover_humble .
```

## 2. Allow X11 Forwarding

Run on the host machine:

```bash id="x7f0p8"
xhost +local:docker
```

## 3. Start the Container

```bash id="6r3yl8"
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/ros2_ws \
  leo_rover_humble
```

## 4. Build the Workspace

Inside the container:

```bash id="mq08em"
cd /ros2_ws

colcon build --symlink-install --packages-skip leo_rover_slam

source install/setup.bash
```

---

# Launch Gazebo

```bash id="6s6q0f"
ros2 launch leo_rover_gazebo two_robots.launch.py
```

This spawns:

* `/leo1`
* `/leo2`

---

# Teleoperate a Rover

Open a new terminal inside the container:

```bash id="h7zqkl"
cd /ros2_ws
source install/setup.bash
```

Control `leo1`:

```bash id="qqfj2n"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/leo1/cmd_vel
```

---

# Launch RViz2

```bash id="sv3g8k"
rviz2 --ros-args -p use_sim_time:=true
```

In RViz:

## Global Options

Set:

```text id="wsf6xp"
Fixed Frame = map
```

## Add Displays

### LaserScan

Topic:

```text id="89lj5x"
/leo1/scan
```

### Map

Topic:

```text id="0x0d0m"
/map
```

---

# Run SLAM Toolbox

Open another terminal:

```bash id="m6h11m"
cd /ros2_ws
source install/setup.bash
```

Run:

```bash id="n0c1xm"
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true \
  -p odom_frame:=leo1/odom \
  -p base_frame:=leo1/base_footprint \
  -p map_frame:=map \
  -p provide_odom_frame:=false \
  -p transform_timeout:=1.0 \
  -r /scan:=/leo1/scan
```

Drive the rover around using the teleop terminal to generate the map.

---

# Save the Map

```bash id="r8kk6y"
ros2 run nav2_map_server map_saver_cli \
  -f /ros2_ws/my_gazebo_map \
  --ros-args -p use_sim_time:=true
```

This generates:

```text id="7cjlwm"
my_gazebo_map.pgm
my_gazebo_map.yaml
```

---

# Launch Nav2

```bash id="yvlc2x"
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  namespace:=leo1 \
  use_namespace:=True \
  map:=/ros2_ws/my_gazebo_map.yaml \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```
