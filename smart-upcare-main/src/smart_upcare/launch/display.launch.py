#!/usr/bin/env python3
# display.launch.py
# Gazebo 없이 RViz로 로봇 모양만 확인 (가벼움, URDF 검증용)
# 실행: ros2 launch smart_upcare display.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('smart_upcare')
    xacro_file = os.path.join(pkg, 'urdf', 'smart_upcare.urdf.xacro')

    # xacro → URDF 변환 (실행 시점에 자동으로)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1) robot_state_publisher: URDF의 TF(고정 조인트들) 발행
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 2) joint_state_publisher_gui: 조인트 슬라이더 (지금은 다 fixed라 크게 쓸 건 없지만
    #    나중에 리프트 prismatic 추가하면 여기서 리프트 높이 슬라이더로 테스트 가능)
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # 3) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([rsp, jsp, rviz])
