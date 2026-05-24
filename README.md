# Leo Rover Gazebo Simulation

## Requirements

* Ubuntu 22.04 (Jammy)
* ROS 2 Humble
* Gazebo Harmonic (`gz-sim8`)

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

## 4. Build the Workspace

Inside the container:

```bash
cd /ros2_ws
colcon build --symlink-install --packages-skip leo_rover_slam
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
cd /ros2_ws
source install/setup.bash
```

Run:

```bash
ros2 launch leo_rover_gazebo slam.launch.py
```


# Launch Nav2

```bash
 ros2 launch leo_rover_gazebo nav2.launch.py
```
