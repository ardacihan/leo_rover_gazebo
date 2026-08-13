cmake_minimum_required(VERSION 3.8)
project(leo_rover_real_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/depth_height_filter.py
  scripts/mapping_artifact_recorder.py
  scripts/scan_fusion.py
  scripts/safety_command_gate.py
  scripts/safe_room_explorer.py
  scripts/wheel_odom_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/exploration_policy.py
  scripts/mapping_artifacts.py
  scripts/sensor_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
