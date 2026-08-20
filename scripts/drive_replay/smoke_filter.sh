#!/bin/bash
# 45 s check: does the cloud filter produce a sane filtered stream?
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=/mnt/c/Users/smirn/Desktop/leo_rover_gazebo
source /opt/ros/humble/setup.bash
source /home/smirn/leo_ws/install/setup.bash
bash "$REPO/scripts/leo_cleanup_wsl.sh" >/dev/null
python3 "$HERE/depth_to_points.py" --ros-args -p use_sim_time:=true >/tmp/d2p.log 2>&1 &
ros2 run leo_nav2_exploration cloud_filter --ros-args -p use_sim_time:=true >/tmp/cf.log 2>&1 &
sleep 3
# skip the parked start: begin where the rover is moving with objects around
timeout --signal=INT 45 ros2 bag play ~/bags/drive_2026-08-20 --clock 50 \
    >/dev/null 2>&1 &
sleep 25
python3 - <<'EOF'
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n = Node('probe', parameter_overrides=[])
counts = {'raw': [], 'filt': []}
n.create_subscription(PointCloud2, '/camera/camera/depth/color/points',
                      lambda m: counts['raw'].append(m.width), qos_profile_sensor_data)
n.create_subscription(PointCloud2, '/camera_points_filtered',
                      lambda m: counts['filt'].append(m.width), qos_profile_sensor_data)
end = time.time() + 12
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.3)
print('raw msgs', len(counts['raw']), 'avg pts', sum(counts['raw'])//max(len(counts['raw']),1))
print('filtered msgs', len(counts['filt']), 'avg pts', sum(counts['filt'])//max(len(counts['filt']),1))
EOF
bash "$REPO/scripts/leo_cleanup_wsl.sh" >/dev/null
tail -3 /tmp/cf.log
