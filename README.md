# Leo Rover Gazebo Simulation

## Requirements
*   **OS:** Ubuntu 24.04 (Noble)
*   **ROS 2:** Jazzy
*   **Dependencies:**
    ```bash
    sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-xacro
    ```

---

## Build

```bash
# Remove the directories
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## Run Simulation

Spawn multiple robots with a single command. Each rover will be isolated in its own namespace (e.g., `/leo1`, `/leo2`).

```bash
ros2 launch leo_rover_gazebo two_robots.launch.py num_robots:=3
```

**Default:** `num_robots:=2`