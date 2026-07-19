cmake_minimum_required(VERSION 3.8)
project(smart_bringup)

find_package(ament_cmake REQUIRED)

# 통합 런치만 설치 (컴파일 없음)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
