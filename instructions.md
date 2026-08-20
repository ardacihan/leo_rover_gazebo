# Generate a new map (real Leo Rover)

Jetson with ROS 2 Humble. Example: `jetson-01`, `ROS_DOMAIN_ID=1`.

## 1. Build
```bash
git pull
source /opt/ros/humble/setup.bash
colcon build --packages-select leo_rover_real_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=1
```

## 2. Robot + LIDAR
Do not start a second bringup/LIDAR if already running.
```bash
ros2 launch leo_bringup leo_bringup.launch.xml \
  wheel_odom:=/wheel_odom publish_odom_tf:=false
```
Confirm fresh `/scan`, `/wheel_odom`, battery, and no foreign `/cmd_vel` publisher.

## 3. SLAM (no explorer yet)
```bash
ros2 launch leo_rover_real_bringup safe_mapping.launch.py \
  start_explorer:=false
```
Confirm `/map` is live.

## 4. Move
Teleop must publish to `/cmd_vel_request`, never `/cmd_vel`. Or:
```bash
ros2 run leo_rover_real_bringup safe_room_explorer.py --ros-args \
  -p run_duration:=30.0 -p max_distance:=2.0
```
Path: request → `safety_command_gate` → `/cmd_vel_raw` → `collision_monitor` → `/cmd_vel`.

## 5. Save (while SLAM is still up)
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/room_map_$(date +%Y%m%d_%H%M) \
  --ros-args -p save_map_timeout:=20.0
```
Zero velocity, then stop the mapper.