#!/usr/bin/env python3
"""aruco_docking 브링업 — estimator + dock_controller (워크스테이션에서 실행).

    ros2 launch aruco_docking dock.launch.py

- aruco_estimator: /camera/image_raw/compressed(+camera_info) → /detected_dock_pose
    compressed 구독(WiFi 대역폭). 카메라는 로봇 Pi(smart_robot_bringup)에서 발행.
- dock_controller: /detected_dock_pose + TF(camera→base_link) + /odom → /cmd_vel
    확정 게인 기본값. /start_work_dock(전진)·/start_home_dock(후진)·/start_undock 서비스.
    auto_undock_delay=0(FSM 분리 호출).

전제: 같은 ROS_DOMAIN_ID(AMR1=30/AMR2=31)로 로봇 Pi와 통신. static TF·odom은 Pi가 발행.
      노드 코드는 도메인 무관 — 이 launch를 해당 도메인 환경에서 띄우면 됨.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # 게인 실험용. dock_controller는 파라미터 콜백이 없어 `ros2 param set`이 안 먹는다(§8.6)
        #   → 기동 시점에 넣어야 실효. 기본값 = 노드 기본값(k_y=6.0)과 동일.
        DeclareLaunchArgument('k_y', default_value='6.0'),
        # 도크별 마커 법선 오프셋(deg). 정지 관측 e_θ 를 그대로 넣는다. 0 = 보정 없음.
        DeclareLaunchArgument('marker_yaw_offset_deg', default_value='0.0'),
        # 재배치 스킵 임계(m). 이 이내면 PREALIGN 생략 → SERVO 직행(스킵 케이스).
        DeclareLaunchArgument('prealign_min_ey', default_value='0.010'),
        # 마커축 ↔ 슬롯중심 측면 오프셋(m). 실물 안착 편차를 재서 넣는다(밀리는 반대 부호).
        DeclareLaunchArgument('target_lateral_offset', default_value='0.0'),
        Node(
            package='aruco_docking', executable='aruco_estimator', name='aruco_estimator',
            output='screen',
            parameters=[{'use_compressed': True, 'process_rate_hz': 10.0}],
            remappings=[
                ('image_raw', '/camera/image_raw/compressed'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),
        Node(
            package='aruco_docking', executable='dock_controller', name='dock_controller',
            output='screen',
            # 확정 게인은 노드 기본값. auto_undock_delay=0(FSM이 undock 분리 호출).
            parameters=[{
                'auto_undock_delay': 0.0,
                'k_y': ParameterValue(LaunchConfiguration('k_y'), value_type=float),
                'marker_yaw_offset_deg': ParameterValue(
                    LaunchConfiguration('marker_yaw_offset_deg'), value_type=float),
                'prealign_min_ey': ParameterValue(
                    LaunchConfiguration('prealign_min_ey'), value_type=float),
                'target_lateral_offset': ParameterValue(
                    LaunchConfiguration('target_lateral_offset'), value_type=float),
            }],
            # odom = raw 휠(IMU 없음, 바닥 슬립 시 yaw 틀어짐) → EKF 융합 출력으로.
            #   PREALIGN·CREEP·UNDOCK의 회전(yaw) 정밀도 개선. (EKF는 smart_robot_bringup에서 발행)
            remappings=[('odom', '/odometry/filtered')],
        ),
        # work(전진)은 staging 방식으로 dock_controller에 통합됨(2026-07-17). 별도 노드 없음.
    ])
