# Debug tooling for real-rover exploration runs

Session tools developed on jetson-04 (2026-08-14). All run ON the Jetson with
`source /opt/ros/humble/setup.bash` and the robot's `ROS_DOMAIN_ID` exported.
Copy with `scp`, then strip CRLF (`sed -i 's/\r$//' <file>`) — see the CRLF
warning in REAL_ROVER_GUIDE.md.

## firmware_stability_monitor.py

Watches firmware health for 300 s: battery-telemetry rate per 30 s bin,
battery voltage, and `enP8p1s0` traffic to/from the rover SBC. Baselines:
idle ~24 KB/s; full mapping stack ~85 KB/s; the documented firmware-starvation
incident was ~1400 KB/s. Battery telemetry must hold 10 Hz in every bin.

    nohup bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=4; \
      exec python3 firmware_stability_monitor.py' >/tmp/fw_monitor.log 2>&1 &

Note: firmware topics are best-effort QoS and `/rob_2/firmware/wheel_odom` is
`leo_msgs/WheelOdom`, NOT `nav_msgs/Odometry` — this script intentionally only
counts battery messages for liveness. `ros2 topic hz` is the reliable probe.

## debug_color_throttle.py

Republishes `/camera/camera/color/image_raw` at max 5 Hz on `/debug/color_5hz`
(plus camera_info). Record THIS in rosbags, never the raw image topics:
bagging raw 640x480 color+depth pushed load average to 9.3/6-cores and caused
0.4-0.5 s scan arrival gaps that tripped the explorer's watchdogs.

## render_labeled_debug_video.py

Renders an analysis video from an exploration rosbag (edit the BAG/OUT paths
at the top): camera frames + decision banner (DRIVING / CM SLOWDOWN /
OBSTACLE: CM HOLDING) derived from `/cmd_vel_request` vs `/cmd_vel`, top-down
fused-LIDAR + camera-scan panel with the collision footprint, explorer
mode/battery from `/rosout`, path length, gate closure reasons. ~2000 frames
render in ~4 min on the Jetson. Requires the bag to contain the topic set
recorded by the standard debug bag command (see REAL_ROVER_GUIDE.md
exploration runbook).

## Standard debug bag topic set

Everything except `/cmd_vel_raw` (recording it trips the gate's anti-bypass
audit and closes the gate) and except raw image topics (CPU):

    ros2 bag record -o ~/leo_bags/<name> \
      /scan /scan_lidar_base /scan_collision_fused /scan_slam_fused \
      /camera/scan_collision /camera/scan_slam \
      /cmd_vel_request /cmd_vel /rob_2/cmd_vel \
      /map /exploration_path /tf /tf_static \
      /wheel_odom /rob_2/firmware/wheel_odom /rob_2/firmware/imu \
      /rob_2/firmware/battery_averaged \
      /collision_monitor/approach_footprint /collision_monitor/footprint \
      /collision_monitor/transition_event /rosout \
      /debug/color_5hz /debug/color_5hz/camera_info
