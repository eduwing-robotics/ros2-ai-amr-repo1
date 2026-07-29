# rosdep 의존성 설치 목록

`rosdep install --from-paths src --ignore-src -r -y` 로 설치되는 외부 의존성.
(각 패키지 `package.xml`의 `<depend>` 기준 · `--ignore-src`로 우리 패키지·서드파티 소스는 제외)

| 분류 | 패키지 |
|---|---|
| **ROS 2 코어** | rclcpp · rclpy · std_msgs · std_srvs · geometry_msgs · sensor_msgs · nav_msgs · action_msgs · rcl_interfaces · tf2_ros · tf2_geometry_msgs · launch · launch_ros · ament_cmake · ament_index_python |
| **Nav2 스택** | nav2_bringup · nav2_common · nav2_msgs · rviz2 |
| **Behavior Tree** | behaviortree_cpp |
| **로컬라이제이션** | robot_localization (EKF) |
| **도메인 브릿지** | domain_bridge |
| **비전 / 카메라** | cv_bridge · camera_ros |
| **파이썬 라이브러리** | python3-numpy · python3-opencv · python3-psycopg2 · python3-pyqt5 · python3-yaml |
| **테스트 / 린트** | ament_copyright · ament_flake8 · ament_pep257 · pytest |
