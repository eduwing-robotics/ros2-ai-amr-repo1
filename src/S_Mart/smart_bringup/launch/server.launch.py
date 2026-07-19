"""서버 통합 런치 — 데탑(도메인 12)의 ROS 노드를 한 번에 기동.

    ros2 launch smart_bringup server.launch.py
    ros2 launch smart_bringup server.launch.py enable_ai:=false   # 순수 주행/취소 테스트
    ros2 launch smart_bringup server.launch.py conf:=0.5          # ai 감지 문턱

묶는 것 (전부 데탑·도메인 12):
  traffic_node · domain_bridge · fleet_manager · task_manager · ai_bringup

★ ROS 노드만 묶는다. 아래 둘은 ROS 프로세스가 아니라 별도 실행:
    서버(FastAPI):  cd server && source .venv/bin/activate && uvicorn app.main:app --host 0.0.0.0 --port 8000
    고객 웹 UI:     cd client && npm run dev
※ 카메라 브링업(camera_bringup)은 노트북에서 별도 실행(카메라가 물리적으로 그쪽).
※ 전제: PostgreSQL(s_mart @ localhost) 기동 상태. fleet/task/서버가 이 DB에 붙는다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_launch = os.path.join(
        get_package_share_directory('smart_domain_bridge'), 'launch', 'bridge.launch.py')
    ai_launch = os.path.join(
        get_package_share_directory('ai_detector'), 'launch', 'ai_bringup.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('enable_ai', default_value='true',
                              description='ai_bringup(입출고 감지) 포함 여부 (순수 주행 테스트 시 false)'),
        DeclareLaunchArgument('conf', default_value='0.35',
                              description='ai 감지 신뢰도 문턱 (ai_bringup으로 전달)'),

        # ── 교통 관리 ──
        Node(package='traffic_manager', executable='traffic_node',
             name='traffic_node', output='screen'),

        # ── 도메인 브릿지 (12 ↔ 30/31) ──
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),

        # ── 임무 배정 ──
        Node(package='fleet_manager', executable='fleet_manager',
             name='fleet_manager', output='screen'),

        # ── 임무 생성 ──
        Node(package='task_manager', executable='task_manager',
             name='task_manager', output='screen'),

        # ── AI 입출고 감지 (enable_ai=false로 제외 가능) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ai_launch),
            condition=IfCondition(LaunchConfiguration('enable_ai')),
            launch_arguments={'conf': LaunchConfiguration('conf')}.items(),
        ),
    ])
