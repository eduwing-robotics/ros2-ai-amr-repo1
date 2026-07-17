"""사람 감지 브링업 (데탑 = GPU, 로봇 도메인) — 도킹 노드와 동일 배치.

카메라는 로봇 Pi에서 발행(/camera/image_raw/compressed), 추론은 여기서.
    ros2 launch human_detector human_detector.launch.py
    ros2 launch human_detector human_detector.launch.py conf:=0.5 release_sec:=3.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('conf', default_value='0.35',
                              description='사람 감지 신뢰도 문턱'),
        DeclareLaunchArgument('release_sec', default_value='2.0',
                              description='해제 지연(초) — 마지막 감지 후 이 시간 미감지시 재개'),
        DeclareLaunchArgument('min_height_ratio', default_value='0.6',
                              description='거리 필터 — 박스높이/화면높이 이상인 가까운 사람만 정지 '
                                          '(0=끔). 0.6=A4가 세로 60% 채울 만큼 가까울 때만 정지. '
                                          'A4 타겟을 정지거리에 두고 debug height_ratio 보고 재조정'),
        DeclareLaunchArgument('camera', default_value='topic:/camera/image_raw/compressed',
                              description='카메라 소스 (로봇 카메라 압축 토픽)'),
        Node(
            package='human_detector',
            executable='human_detector',
            parameters=[{
                'conf': LaunchConfiguration('conf'),
                'release_sec': LaunchConfiguration('release_sec'),
                'min_height_ratio': LaunchConfiguration('min_height_ratio'),
                'camera': LaunchConfiguration('camera'),
            }],
            output='screen',
        ),
    ])
