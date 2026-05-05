cmake_minimum_required(VERSION 3.8)
project(leo_rover_gazebo)

# ROS
find_package(ament_cmake REQUIRED)
find_package(leo_rover_description REQUIRED)

# Gazebo (modern)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)

# -----------------------
# Plugins
# -----------------------

add_library(BasicSystem SHARED src/BasicSystem.cc)
target_include_directories(BasicSystem PRIVATE include)
target_link_libraries(BasicSystem PRIVATE
  gz-sim8::gz-sim8
)

add_library(FullSystem SHARED src/FullSystem.cc)
target_include_directories(FullSystem PRIVATE include)
target_link_libraries(FullSystem PRIVATE
  gz-sim8::gz-sim8
)

# -----------------------
# Install
# -----------------------

install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)



# -----------------------
# Env hooks
# -----------------------




# -----------------------
ament_package()
