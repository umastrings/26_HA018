#!/usr/bin/env python3
"""
spawn_obstacle.launch.py — 실행 중인 Gazebo 에 데모 장애물 투입

  월드에는 장애물이 들어있지 않다(기본 경로 무간섭). 시연 중에 이 launch 로 투입한다.

  실행:
    # 병실 중앙 통로 (회피 + 재계획 데모, 권장)
    ros2 launch smart_upcare_control spawn_obstacle.launch.py x:=11.5 y:=0.0

    # 복도 알코브 구간 (좁은 복도 우회 데모) — 반드시 x 3.6~4.4 범위
    ros2 launch smart_upcare_control spawn_obstacle.launch.py \
        model:=demo_wheelchair name:=wc x:=4.0 y:=-0.30 yaw:=1.5708

    # 복도 직선부 (통과 불가 → Collision Monitor 정지/대기 데모)
    ros2 launch smart_upcare_control spawn_obstacle.launch.py x:=2.0 y:=-0.30

  제거:
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'demo_obstacle'}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('smart_upcare')

    model = LaunchConfiguration('model')   # demo_box | demo_wheelchair
    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    sdf_path = PathJoinSubstitution([
        pkg_desc, 'worlds', 'models',
        [model, '.sdf'],
    ])

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='demo_box',
                              description='demo_box 또는 demo_wheelchair'),
        DeclareLaunchArgument('name', default_value='demo_obstacle',
                              description='Gazebo entity 이름 (삭제 시 필요)'),
        DeclareLaunchArgument('x', default_value='11.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-file', sdf_path,
                '-entity', name,
                '-x', x, '-y', y, '-z', '0.0', '-Y', yaw,
            ],
        ),
    ])
