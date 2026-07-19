"""도킹+사람감지 통합 런치 — 데탑에서 '로봇 도메인'으로 기동 (GPU 추론).

로봇당 한 벌씩, ROS_DOMAIN_ID를 로봇 것으로 바꿔 실행:
    ROS_DOMAIN_ID=30 ros2 launch smart_bringup perception.launch.py   # AMR_1
    ROS_DOMAIN_ID=31 ros2 launch smart_bringup perception.launch.py   # AMR_2

묶는 것 (둘 다 데탑 하드웨어, 로봇 카메라 토픽 구독):
  aruco_docking dock.launch.py   — 추정(estimator)+제어(controller) 도킹 노드
  human_detector                 — 사람 감지 → /human_stop

※ 카메라는 로봇 Pi에서 발행(/camera/image_raw/compressed). 여기선 추론만.
※ 도킹/사람감지가 같은 카메라 토픽을 구독한다(로봇 도메인이라 브릿지 없이 직접).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dock_launch = os.path.join(
        get_package_share_directory('aruco_docking'), 'launch', 'dock.launch.py')
    human_launch = os.path.join(
        get_package_share_directory('human_detector'), 'launch', 'human_detector.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('enable_human', default_value='true',
                              description='사람 감지 포함 여부 (도킹만 쓸 때 false)'),
        DeclareLaunchArgument('min_height_ratio', default_value='0.6',
                              description='사람 거리 필터 (human_detector로 전달)'),

        # ── 도킹 (추정 + 제어) ──
        IncludeLaunchDescription(PythonLaunchDescriptionSource(dock_launch)),

        # ── 사람 감지 (enable_human=false로 제외 가능) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(human_launch),
            condition=IfCondition(LaunchConfiguration('enable_human')),
            launch_arguments={
                'min_height_ratio': LaunchConfiguration('min_height_ratio'),
            }.items(),
        ),
    ])
