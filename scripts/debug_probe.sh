#!/usr/bin/env bash
docker exec leo_sim bash -lc '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/reports/final_runs/depot_world/live_check --ros-args -p use_sim_time:=true 2>&1 | tail -2
'
