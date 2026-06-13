cmake_minimum_required(VERSION 3.8)
project(leo_rover_gazebo)

find_package(ament_cmake REQUIRED)
find_package(leo_rover_description REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()