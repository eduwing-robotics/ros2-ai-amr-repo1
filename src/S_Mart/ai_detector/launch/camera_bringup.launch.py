"""카메라 브링업 (노트북 = 카메라가 물리적으로 꽂힌 쪽) — 추론은 하지 않는다.

    ros2 launch ai_detector camera_bringup.launch.py

추론은 데탑에서 별도로:  ros2 launch ai_detector ai_bringup.launch.py
두 머신은 ROS_DOMAIN_ID(12)와 RMW_IMPLEMENTATION(rmw_cyclonedds_cpp)이 같아야 한다.

발행 토픽 (추론 노드는 전부 /compressed 쪽을 구독한다):
  IN-1  RealSense D435 → /camera/camera/color/image_raw/compressed
  OUT-1 Vimicro 3420   → /out1/image_raw/compressed
  OUT-2 Generic PC CAM → /out2/image_raw/compressed

※ raw가 아니라 compressed를 쓰는 이유: 640x480 raw는 캠당 921KB/frame(15fps면 14MB/s)
  이고 RealSense까지 더하면 네트워크가 감당 못 한다. JPEG은 프레임이 40배 작다.
  image_transport가 /compressed를 자동으로 같이 발행하므로 별도 노드는 필요 없다.
  (같은 머신에서 다 돌릴 거면 추론 노드에 camera:=topic:<raw토픽> 을 주면 된다)

※ 웹캠 device는 by-id 고정 — 포트를 바꿔 꽂아도 이름이 유지된다.
  지금 3대가 전부 다른 모델이라 by-id로 구분된다. 단 웹캠 2대는 시리얼이 없어
  (SerialNumber=0) by-id 이름이 '제조사+모델명'뿐이다. 같은 모델 2대를 쓰게 되면
  이름이 충돌해 OUT-1/OUT-2가 뒤바뀌므로, 그때는 by-path로 바꿀 것:
      ls -l /dev/v4l/by-path/     (-video-index0 이 캡처 노드, index1 은 메타데이터)
  /dev/videoN 직접 지정은 금물 — 재열거 때 번호가 뒤바뀐다. RealSense가 video0~5를
  잡고 있다가 웹캠을 꽂자 video4~9로 밀리는 것을 확인했다.

※ ★ usb_cam은 심볼릭 링크를 직접 못 받는다 — by-id 링크가 '../../video0'을 가리키는데
  이를 /dev/ 기준으로 붙여 '/dev/../../video0'라는 잘못된 경로를 만들고 실패한다.
  그래서 아래 OpaqueFunction이 기동 시점에 realpath로 풀어서 /dev/videoN 을 넘긴다.
  기동 때마다 다시 푸므로 by-id의 포트 독립성은 그대로다.

※ 웹캠 2대 모두 YUYV만 지원하고 최대 640x480@30/15 (MJPG 없음) — 실측 확인.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_BY_ID = '/dev/v4l/by-id/usb-{}-video-index0'
# 어느 물리 캠이 어느 게이트를 비추는지는 코드가 알 수 없다 — 여기가 유일한 정의 지점.
# 게이트에 물건을 놓고 debug 토픽으로 눈으로 확인해 맞출 것:
#   rqt_image_view /detection/out1/debug --ros-args -p image_transport:=compressed
# 2026-07-16: 실주행에서 OUT-1 배달 물건을 out2 감지 노드가 보고 {slot:OUT-2}를 쏴
#   task_manager가 "awaiting_pickup 주문 없음"으로 버리는 것을 확인 → 아래 둘을 맞바꿈.
_OUT1_ID = 'Generic_USB2.0_PC_CAMERA'    # OUT-1 웹캠
_OUT2_ID = 'Vimicro_Corp._3420'          # OUT-2 웹캠


def _usb_cam(name, device, fps):
    """usb_cam 노드 1대 — /<name>/image_raw(+/compressed) 로 발행.
    device는 realpath로 푼 실제 /dev/videoN 이어야 한다(상단 ★ 주석)."""
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'usb_cam_{name}',
        namespace=name,
        parameters=[{
            'video_device': device,
            # 두 캠 다 YUYV 전용 — yuyv2rgb로 변환해 rgb8로 발행
            'pixel_format': 'yuyv2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': fps,
            'camera_name': name,
            'frame_id': f'{name}_cam',
        }],
        output='screen',
    )


def _webcams(context):
    """by-id 링크를 realpath로 풀어 usb_cam 노드를 만든다 (기동 시점 평가)."""
    fps = float(LaunchConfiguration('webcam_fps').perform(context))
    enable_out2 = LaunchConfiguration('enable_out2').perform(context).lower()

    wanted = [('out1', 'out1_cam')]
    if enable_out2 in ('true', '1', 'yes'):
        wanted.append(('out2', 'out2_cam'))

    resolved = []
    for name, arg in wanted:
        link = LaunchConfiguration(arg).perform(context)
        if not os.path.exists(link):
            raise RuntimeError(
                f'{name.upper()} 웹캠을 찾을 수 없다: {link}\n'
                f'  연결을 확인하거나 by-id 이름을 갱신할 것:  ls -l /dev/v4l/by-id/')
        resolved.append(os.path.realpath(link))

    # 두 캠이 같은 장치로 풀리면 OUT-1/OUT-2 신호가 뒤바뀌어 로봇이 엉뚱한 슬롯을
    # 처리한다. 조용히 틀리느니 기동 때 터뜨린다 (by-id 이름 충돌·인자 오지정 방지).
    if len(set(resolved)) != len(resolved):
        raise RuntimeError(
            f'OUT-1/OUT-2가 같은 장치로 풀렸다: {resolved}\n'
            f'  by-id 이름이 충돌했거나(동일 모델 2대) 인자를 잘못 준 경우다.')

    return [_usb_cam(name, dev, fps)
            for (name, _), dev in zip(wanted, resolved)]


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('out1_cam', default_value=_BY_ID.format(_OUT1_ID),
                              description='OUT-1 웹캠 (by-id 고정 — 상단 주석 참고)'),
        DeclareLaunchArgument('out2_cam', default_value=_BY_ID.format(_OUT2_ID),
                              description='OUT-2 웹캠 (by-id 고정)'),
        DeclareLaunchArgument('enable_out2', default_value='true',
                              description='OUT-2 웹캠 발행 (캠 미연결 시 false)'),
        DeclareLaunchArgument('webcam_fps', default_value='15.0',
                              description='웹캠 fps (640x480은 30/15만 지원). 추론이 5Hz라 15로 충분'),

        # ── IN-1: RealSense D435 ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                # USB2 연결도 커버하는 보수적 프로파일 (감지는 5Hz라 충분)
                'rgb_camera.color_profile': '640x480x15',
                'enable_depth': 'false',       # 입고 감지는 컬러만 — USB 대역폭 절약
                'enable_infra1': 'false',
                'enable_infra2': 'false',
            }.items(),
        ),

        # ── OUT-1 / OUT-2: PC 웹캠 ──
        OpaqueFunction(function=_webcams),
    ])
