cmake_minimum_required(VERSION 3.8)
project(smart_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
