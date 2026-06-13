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

# Launch Gazebo

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py
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
Gazebo + leo1 + leo2 + SLAM + shared map
```

Current mapping flow:

```text
/leo1/scan + /leo1/odom  ->  /leo1/map
/leo2/scan + /leo2/odom  ->  /leo2/map
/leo1/map + /leo2/map    ->  /shared_map
```

Robot 2 is aligned with a fixed simulation offset:

```text
x   = 2.36
y   = -11.27
yaw = 0.0
```

This can be changed in:

```text
src/multi_robot_shared_mapping/launch/shared_mapping_demo.launch.py
```

or overridden at launch:

```bash
ros2 launch multi_robot_shared_mapping shared_mapping_demo.launch.py \
  robot2_to_shared_x:=2.36 \
  robot2_to_shared_y:=-11.27 \
  robot2_to_shared_yaw:=0.0
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
Add -> By topic -> /leo1/map      -> Map1
Add -> By topic -> /leo2/map      -> Map2
Add -> By topic -> /shared_map    -> Map
Add -> By topic -> /leo1/scan     -> LaserScan1
Add -> By topic -> /leo2/scan     -> LaserScan2
Add -> By display type -> TF
```

Optional: rename the map displays in RViz.

Note:
/shared_map is a topic.
RViz Fixed Frame should be leo1/map.


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
ros2 run nav2_map_server map_saver_cli \
  -f /ros2_ws/maps/shared_office_map \
  --ros-args -r map:=/shared_map
```