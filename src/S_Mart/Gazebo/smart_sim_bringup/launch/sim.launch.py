#!/usr/bin/env python3
"""S-Mart Gazebo 시뮬 브링업 — 실기 '로봇 브링업'의 시뮬 대체물.

이거 하나로 실기의 두 로봇 브링업(하드웨어 계층)이 재현된다:
  - Gazebo 월드(실기 맵에서 자동 생성된 smart_arena) + burger 2대 스폰
  - 로봇별 ros_gz_bridge를 '각 로봇 도메인(30/31)에서' 실행
    → 각 도메인에 실기와 동일한 전역 토픽(/scan /odom /imu /cmd_vel) 등장
  - 로봇별 robot_state_publisher + EKF(실기 smart_robot_bringup의 ekf.yaml 재사용)
  - 서버 도메인(12)용 /clock 브리지

이후는 실기와 똑같이 (단, use_sim_time:=true):
  [도메인 30/31 각각] nav2 + robot_fsm
  [도메인 12] traffic_node + domain_bridge + fake_fleet

사용:
  ros2 launch smart_sim_bringup sim.launch.py            # GUI 포함
  ros2 launch smart_sim_bringup sim.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 스폰 위치 — 실기 홈 (config/spawn.yaml과 동일 값, launch에서 직접 사용)
ROBOTS = [
    # (모델, 도메인, x, y, yaw)  — 코를 서쪽(W)으로
    ('burger1', '30', 1.214, -0.064, 3.14159),   # AMR_1 @ N25 (home_A)
    ('burger2', '31', 1.214, 0.236, 3.14159),    # AMR_2 @ N20 (home_B)
]
SERVER_DOMAIN = '12'


def generate_launch_description():
    pkg = get_package_share_directory('smart_sim_bringup')
    tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    robot_bringup = get_package_share_directory('smart_robot_bringup')

    world = os.path.join(pkg, 'worlds', 'smart_arena.sdf')
    ekf_param = os.path.join(robot_bringup, 'param', 'ekf.yaml')
    urdf = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf) as f:
        robot_desc = f.read()

    headless = LaunchConfiguration('headless')

    # gz가 모델(burger1/2 + 원본 메시 turtlebot3_common)을 찾도록
    resource_path = ':'.join([
        os.path.join(pkg, 'models'),
        os.path.join(tb3_gz, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    actions = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='true면 gz GUI 없이 (측정 배치 실행용)'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),

        # Gazebo 서버(+GUI)
        ExecuteProcess(cmd=['gz', 'sim', '-r', world],
                       output='screen', condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world],
                       output='screen', condition=IfCondition(headless)),

        # 서버 도메인(12)용 clock 브리지
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge_server', output='screen',
             additional_env={'ROS_DOMAIN_ID': SERVER_DOMAIN},
             parameters=[{'config_file':
                          os.path.join(pkg, 'config', 'bridge_clock.yaml')}]),
    ]

    for i, (model, domain, x, y, yaw) in enumerate(ROBOTS, start=1):
        env = {'ROS_DOMAIN_ID': domain}
        actions += [
            # 스폰 (gz transport라 도메인 무관)
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-file',
                            os.path.join(pkg, 'models', model, 'model.sdf'),
                            '-name', model,
                            '-x', str(x), '-y', str(y), '-z', '0.01',
                            '-Y', str(yaw)]),
            # 브리지 — 해당 로봇 도메인에서 실기 전역 토픽 재현
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name=f'bridge_{model}', output='screen',
                 additional_env=env,
                 parameters=[{'config_file':
                              os.path.join(pkg, 'config',
                                           f'bridge_robot{i}.yaml')}]),
            # URDF 정적 TF (base_footprint→base_link→base_scan 등)
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 additional_env=env,
                 parameters=[{'robot_description': robot_desc,
                              'use_sim_time': True}]),
            # EKF — 실기와 동일 설정 재사용 (/odom+/imu → odom→base_footprint TF,
            # /odometry/filtered 발행 — nav2가 이걸 먹음)
            Node(package='robot_localization', executable='ekf_node',
                 name='ekf_filter_node', output='screen',
                 additional_env=env,
                 parameters=[ekf_param, {'use_sim_time': True}]),
        ]

    return LaunchDescription(actions)
