"""로봇 통합 런치 — 로봇 Pi에서 하드웨어~FSM을 한 번에 기동.

로봇당 도메인을 로봇 것으로 두고 실행:
    ROS_DOMAIN_ID=30 ros2 launch smart_bringup robot_all.launch.py   # AMR_1
    ROS_DOMAIN_ID=31 ros2 launch smart_bringup robot_all.launch.py   # AMR_2
    ros2 launch smart_bringup robot_all.launch.py usb_port:=/dev/ttyACM1 fork_dev:=/dev/ttyACM0

묶는 것 (전부 로봇 Pi 온보드):
  smart_robot_bringup robot.launch  — TB3 HW + EKF + 카메라 + 도킹 TF
  micro_ros_agent                   — 포크 ESP32 (serial)
  smart_nav_bringup navigation      — nav2 스택
  robot_fsm                         — 임무 실행 FSM

★ 기동 순서: FSM은 순서 무관하게 설계됨 — _auto_init이 AMCL(/initialpose 구독자)을
  무기한 기다리고, travel은 nav 액션서버를 wait_for_server로 기다린다. 그래서 엄격한
  이벤트 시퀀싱은 불필요하다. 다만 nav2는 tf/scan(하드웨어 브링업)이 먼저 올라와 있으면
  기동 로그가 깔끔하므로, nav만 하드웨어 뒤로 nav_delay초 지연을 준다(안전 여유).
  micro_ros와 하드웨어 브링업은 서로 다른 시리얼포트라 동시 기동 OK.

※ 카메라 브링업은 이 robot.launch(smart_robot_bringup) 안에 포함(Pi Cam).
  워크스테이션 도킹/사람감지 추론은 별도(docking.launch.py, 데탑).
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_bringup = PathJoinSubstitution(
        [FindPackageShare('smart_robot_bringup'), 'launch', 'robot.launch.py'])
    nav_bringup = PathJoinSubstitution(
        [FindPackageShare('smart_nav_bringup'), 'launch', 'navigation.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM1',
                              description='TB3(OpenCR) 시리얼 포트'),
        DeclareLaunchArgument('fork_dev', default_value='/dev/ttyACM0',
                              description='포크 ESP32(micro-ROS) 시리얼 포트'),
        DeclareLaunchArgument('fork_baud', default_value='115200',
                              description='포크 ESP32 보드레이트'),
        DeclareLaunchArgument('nav_delay', default_value='10.0',
                              description='하드웨어 기동 후 nav2 기동까지 지연(초) — tf/scan 안정 여유'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='nav2 rviz 표시 (로봇 Pi에선 보통 false)'),

        # ── t=0: 하드웨어 브링업 (TB3 HW + EKF + 카메라 + 도킹 TF) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_bringup),
            launch_arguments={'usb_port': LaunchConfiguration('usb_port')}.items(),
        ),

        # ── t=0: 포크 ESP32 micro-ROS agent (하드웨어 브링업과 다른 포트라 동시 OK) ──
        Node(
            package='micro_ros_agent', executable='micro_ros_agent',
            name='micro_ros_agent', output='screen',
            arguments=['serial', '--dev', LaunchConfiguration('fork_dev'),
                       '-b', LaunchConfiguration('fork_baud')],
        ),

        # ── t=0: FSM (순서 무관 — AMCL·nav 액션서버를 자체 대기) ──
        Node(package='robot_fsm', executable='robot_fsm',
             name='robot_fsm', output='screen'),

        # ── t=nav_delay: nav2 (하드웨어 tf/scan 안정 후) ──
        TimerAction(
            period=LaunchConfiguration('nav_delay'),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_bringup),
                launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
            )],
        ),
    ])
