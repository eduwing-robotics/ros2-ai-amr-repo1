#!/usr/bin/env python3
"""S-Mart nav2 실행 launch.

벤더(turtlebot3) launch를 대체 — 우리 params/map/rviz로 nav2를 실행한다.
nav2_bringup의 bringup_launch.py(맵서버+AMCL+nav2 스택)를 감싸고,
rviz는 use_rviz 인자로 켜고 끌 수 있다. (저사양 서버에서 CPU 아끼려면 use_rviz:=false)

기본값은 전부 이 패키지 안의 파일을 가리키므로 인자 없이도 실행된다:
    ros2 launch smart_nav_bringup navigation.launch.py
    ros2 launch smart_nav_bringup navigation.launch.py use_rviz:=false   # rviz 없이(서버 CPU 절약)
    ros2 launch smart_nav_bringup navigation.launch.py map:=/other/map.yaml params_file:=/other/params.yaml
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
    pkg_share = get_package_share_directory('smart_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 이 패키지 안의 기본 파일 경로
    default_map = os.path.join(pkg_share, 'maps', 'map_gimp.yaml')
    default_params = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav2_view.rviz')

    # 런치 인자
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='시뮬레이션(Gazebo) 클럭 사용 여부 — 실물 로봇은 false'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='맵 yaml 경로 (기본: 패키지 내 map_gimp — origin 보정본)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='nav2 파라미터 yaml 경로 (기본: 패키지 내 nav2_params)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='nav2 라이프사이클 자동 활성화 여부'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='rviz 실행 여부 — 저사양 서버 CPU 절약하려면 false'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='rviz 설정 파일 경로'),
    ]

    # nav2 스택 (map_server + amcl + planner + controller + bt + collision_monitor ...)
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    # rviz (use_rviz=true 일 때만)
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(declare_args + [nav2_stack, rviz])
