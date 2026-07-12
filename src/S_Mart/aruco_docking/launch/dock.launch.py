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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
            parameters=[{'auto_undock_delay': 0.0}],
        ),
    ])
