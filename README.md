# Leo Rover Gazebo Simulation

## Requirements
*   **OS:** Ubuntu 22.04 (Jammy)
*   **ROS 2:** Humble
*   **Gazebo:** Harmonic (gz-sim8)

### 1. Add the Gazebo Harmonic apt repository
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

### 2. Install dependencies
```bash
sudo apt install \
  gz-harmonic \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-interfaces \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-sdformat-urdf \
  libgz-cmake3-dev \
  libgz-plugin2-dev \
  libgz-common5-dev \
  libgz-sim8-dev
```

### 3. Install Leo Rover description package
```bash
# Option A – via apt (if available in your mirror):
sudo apt install ros-humble-leo-description

# Option B – from source:
mkdir -p ~/leo_ws/src
git clone -b humble https://github.com/LeoRover/leo_common-ros2.git ~/leo_ws/src/leo_common
cd ~/leo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select leo_description
source install/setup.bash
```

---

## Build

```bash
# Remove stale artifacts
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
# If you installed leo_description from source, source that workspace first:
# source ~/leo_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run Simulation

Spawns multiple rovers, each isolated in its own namespace (e.g., `/leo1`, `/leo2`, `/leo3`).

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py
```

**Default:** `num_robots = 3` (set at the top of `two_robots.launch.py`).

---

## Docker (recommended for development)

```bash
# Build the image
docker build -t leo_rover_humble .

# Run with display forwarding (Linux host)
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/ros2_ws \
  leo_rover_humble

# Inside the container – build and launch:
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch leo_rover_gazebo two_robots.launch.py
```

## Notes

When in container use ```bash ros2 topic echo </leoX/topic/name> --qos-reliability best_effort```
