cmake_minimum_required(VERSION 3.8)
project(smart_nav_bringup)

find_package(ament_cmake REQUIRED)

# launch·params·map·rviz를 share/에 설치 (self-contained bringup)
install(DIRECTORY
  launch
  params
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# nav2 도달 테스트 노드 (python). ros2 run smart_nav_bringup nav2_node_goal
install(PROGRAMS
  scripts/nav2_node_goal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
