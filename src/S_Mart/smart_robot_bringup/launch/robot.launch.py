#!/usr/bin/env python3
"""S-Mart 로봇 브링업 — TB3 HW + EKF + 카메라 + 도킹 TF 한 방 기동 (로봇 Pi에서 실행).

    ros2 launch smart_robot_bringup robot.launch.py

구성 (전부 Pi 온보드):
  - turtlebot3_bringup robot.launch.py (LDS 라이다 + turtlebot3_node)
      단, 파라미터 burger_ekf.yaml — TB3 자체 odom TF OFF, 휠+IMU 융합 OFF (EKF 전담)
  - robot_localization ekf_node: /odom(휠) + /imu 융합 → odom→base_footprint TF
  - camera_ros: Pi Cam (IMX219) 1024x768 → /camera/image_raw(+compressed)/camera_info
      ★ 캘리브는 ROS_DOMAIN_ID로 선택 (30→AFL1, 31→AFL2). 미설정이면 camera_ros 기본(~/.ros).
  - static TF 2단: base_link→camera_link(실측 x=0.045 z=0.145) → camera(REP-103 광학회전)
      (TB3 URDF 안 건드리고 우리 launch에서 발행 — docs_hub/context/11 §14.10.2)

전제: TURTLEBOT3_MODEL=burger, LDS_MODEL 환경변수, robot_localization·camera_ros 설치,
      Pi에 image-transport-plugins(compressed advertise). 워크스테이션 estimator/controller는
      별도(aruco_docking dock.launch.py). aruco_docking 노드는 도메인 무관.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('smart_robot_bringup')
    tb3_bringup = get_package_share_directory('turtlebot3_bringup')

    burger_param = os.path.join(pkg_share, 'param', 'burger_ekf.yaml')
    ekf_param = os.path.join(pkg_share, 'param', 'ekf.yaml')

    # ── 도메인별 카메라 캘리브 선택 (30=AFL1, 31=AFL2; 미지원 시 ~/.ros 폴백) ──
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    cal_map = {'30': 'AFL1.yaml', '31': 'AFL2.yaml'}
    cam_params = {
        'camera': 0,
        'sensor_mode': '1640:1232',
        'width': 1024,
        'height': 768,
        'format': 'RGB888',
        # ★ 발행 10fps 고정 (libcamera FrameDurationLimits, μs). 100000μs=10fps.
        #   WiFi 대역폭 절약 — 기본 ~30fps compressed 스트림이 Nav2 주행 트래픽과 경쟁해
        #   estimator 이미지 구독 wedge(context/13 §8.5). estimator 처리도 10Hz라 무손실.
        'FrameDurationLimits': [100000, 100000],
    }
    cal_name = cal_map.get(domain)
    if cal_name:
        cal_path = os.path.join(pkg_share, 'config', 'camera_info', cal_name)
        if os.path.exists(cal_path):
            cam_params['camera_info_url'] = 'file://' + cal_path

    return LaunchDescription([
        # ── TB3 하드웨어 (LDS + turtlebot3_node) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup, 'launch', 'robot.launch.py')),
            launch_arguments={'tb3_param_dir': burger_param}.items(),
        ),
        # ── EKF: odom+imu 융합 → odom→base_footprint ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param],
        ),
        # ── 카메라 (camera_ros composable, 도메인별 캘리브) ──
        ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_ros',
                    plugin='camera::CameraNode',
                    parameters=[cam_params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),
        # ── 카메라 static TF 체인 (base_link → camera_link → camera) ──
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_base_link_to_camera_link',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'camera_link',
                       '--x', '0.045', '--y', '0.0', '--z', '0.145'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_camera_link_to_camera',
            arguments=['--frame-id', 'camera_link', '--child-frame-id', 'camera',
                       '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708'],
        ),
    ])
