cmake_minimum_required(VERSION 3.8)
project(smart_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(PROGRAMS
  tools/gen_world.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
