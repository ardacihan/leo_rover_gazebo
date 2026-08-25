# jetson-02 (rover 2) deployment pack — set up 2026-08-25

Copies of the operational scripts deployed to jetson-02:~/leo_nav2_ws.
Rover-2 differences from rover 4: ROS_DOMAIN_ID=2, firmware topics at
root (odom_relay.py bridges /firmware/wheel_odom -> /wheel_odom, no
leo-nav-bridge), camera under /rob_2, RPLIDAR C1 on /dev/ttyUSB0 with
frame renamed laser_frame (run_lidar.sh) so rover-4 configs run as-is.

STATUS: sensor/SLAM bringup validated (scan 10 Hz, odom 20 Hz, map
publishing). OPEN BUG: controller instantly reports "Reached the goal!"
and the rover never moves; collision monitor logs laser_frame TF timing
errors. See docs/ROVER_FAILURE_RUNBOOK.md §12.
