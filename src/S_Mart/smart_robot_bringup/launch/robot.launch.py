#!/usr/bin/env python3
"""S-Mart 로봇 브링업 — TB3 하드웨어 + EKF 한 방 기동 (로봇 Pi에서 실행).

    ros2 launch smart_robot_bringup robot.launch.py

구성:
  - turtlebot3_bringup robot.launch.py (LDS 라이다 + turtlebot3_node)
      단, 파라미터를 burger_ekf.yaml로 교체 — TB3 자체 odom TF 발행 OFF,
      휠 오도메트리 IMU 융합 OFF (전부 EKF가 전담)
  - robot_localization ekf_node: /odom(휠 속도) + /imu(yaw·각속도) 융합
      → odom→base_footprint TF 발행

전제: TURTLEBOT3_MODEL=burger, LDS_MODEL 환경변수 (TB3 표준),
      robot_localization 설치 (sudo apt install ros-jazzy-robot-localization)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('smart_robot_bringup')
    tb3_bringup = get_package_share_directory('turtlebot3_bringup')

    burger_param = os.path.join(pkg_share, 'param', 'burger_ekf.yaml')
    ekf_param = os.path.join(pkg_share, 'param', 'ekf.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup, 'launch', 'robot.launch.py')),
            launch_arguments={'tb3_param_dir': burger_param}.items(),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param],
        ),
    ])
