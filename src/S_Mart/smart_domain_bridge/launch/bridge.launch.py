#!/usr/bin/env python3
"""domain_bridge 실행 — 서버(12) ↔ 로봇1(30)/로봇2(31) 조율 토픽 연결.

config/bridge.yaml 의 토픽 매핑대로 도메인 간 브릿지.
서버 컴퓨터에서 실행 (여러 도메인을 한 프로세스가 중계).

    ros2 launch smart_domain_bridge bridge.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('smart_domain_bridge')
    default_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='domain_bridge 설정 yaml'),
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='smart_domain_bridge',
            arguments=[config],
            output='screen',
        ),
    ])
