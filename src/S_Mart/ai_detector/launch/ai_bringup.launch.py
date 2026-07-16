"""AI 추론 브링업 (데탑 = GPU 있는 쪽) — 카메라는 열지 않는다.

카메라 브링업은 노트북에서 별도로:  ros2 launch ai_detector camera_bringup.launch.py
두 머신은 ROS_DOMAIN_ID(12)와 RMW_IMPLEMENTATION(rmw_cyclonedds_cpp)이 같아야 한다.

    ros2 launch ai_detector ai_bringup.launch.py
    ros2 launch ai_detector ai_bringup.launch.py conf:=0.5           # 문턱 조정
    ros2 launch ai_detector ai_bringup.launch.py enable_out2:=false  # OUT-2 캠 빼고

구독/발행 (전부 서버 도메인 12, 브릿지 불필요):
  IN-1  /camera/camera/color/image_raw/compressed → inbound_detector  → /detection/inbound
  OUT-1 /out1/image_raw/compressed                → outbound_detector → /detection/pickup 등
  OUT-2 /out2/image_raw/compressed                → outbound_detector

※ raw가 아니라 compressed를 구독하는 이유는 camera_bringup.launch.py 주석 참고
  (640x480 raw = 캠당 27MB/s → 3대면 네트워크가 감당 못 함. JPEG은 40배 작다).
  카메라와 추론을 같은 머신에서 돌릴 거면 in_cam/out1_cam/out2_cam 에
  raw 토픽(topic:/out1/image_raw)을 주면 디코드 비용을 아낄 수 있다.
※ 관제 GUI(PyQt)는 /detection/*/debug/compressed 를 직접 구독하면 됨(웹 브릿지 불필요).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    conf = LaunchConfiguration('conf')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('conf', default_value='0.35',
                              description='감지 신뢰도 문턱 (게이트 마운트 후 0.5+ 권장)'),
        DeclareLaunchArgument('rate', default_value='5.0',
                              description='처리 주기 Hz. debug 영상 부드럽게 보려면 15 (GPU 4ms/frame)'),
        DeclareLaunchArgument(
            'in_cam', default_value='topic:/camera/camera/color/image_raw/compressed',
            description='입고 입력 — 노트북 RealSense 브링업이 발행'),
        DeclareLaunchArgument('out1_cam', default_value='topic:/out1/image_raw/compressed',
                              description='OUT-1 입력 — 노트북 usb_cam이 발행'),
        DeclareLaunchArgument('out2_cam', default_value='topic:/out2/image_raw/compressed',
                              description='OUT-2 입력 — 노트북 usb_cam이 발행'),
        DeclareLaunchArgument('enable_out2', default_value='true',
                              description='OUT-2 감지 활성화 (캠 미연결 시 false)'),
        DeclareLaunchArgument(
            'in_clear_frames', default_value='40',
            description='입고 재무장 조건 — 연속 빈 프레임 수 (rate=5면 40=8초). '
                        '로봇이 물건을 드는 동안 3~4초씩 가려져 박스가 깜빡이므로, '
                        '짧으면 집기 도중 재무장 → 같은 물건 재발행 → 중복 입고 task'),

        # ── 입고 ──
        # clear_frames만 출고(기본 10=2초)보다 길게 간다. 출고의 clear_frames는
        # 수령 판정 지연에 직결돼(고객이 가져간 걸 늦게 알아챔) 같이 늘리면 안 된다.
        Node(
            package='ai_detector',
            executable='inbound_detector',
            parameters=[{'conf': conf, 'rate': rate,
                         'camera': LaunchConfiguration('in_cam'),
                         'clear_frames': LaunchConfiguration('in_clear_frames')}],
            output='screen',
        ),

        # ── 출고 OUT-1 ──
        Node(
            package='ai_detector',
            executable='outbound_detector',
            name='outbound_detector_out1',
            parameters=[{'conf': conf, 'rate': rate, 'slot': 'OUT-1',
                         'camera': LaunchConfiguration('out1_cam')}],
            output='screen',
        ),

        # ── 출고 OUT-2 ──
        Node(
            package='ai_detector',
            executable='outbound_detector',
            name='outbound_detector_out2',
            condition=IfCondition(LaunchConfiguration('enable_out2')),
            parameters=[{'conf': conf, 'rate': rate, 'slot': 'OUT-2',
                         'camera': LaunchConfiguration('out2_cam')}],
            output='screen',
        ),
    ])
