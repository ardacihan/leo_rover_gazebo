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
ros2 launch leo_rover_gazebo two_robots.launch.py

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
