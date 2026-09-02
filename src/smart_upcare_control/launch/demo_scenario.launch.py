#!/usr/bin/env python3
"""
demo_scenario.launch.py — 데모 시퀀스 실행

  전제: robot_bringup + navigation 이 이미 떠 있어야 함(위치추정+Nav2).
  이 launch는 시퀀스 오케스트레이터(demo_scenario_node)만 올린다.

  실행:
    # 터미널 1 (실물)
    ros2 launch smart_upcare_control robot_bringup.launch.py
    ros2 launch smart_upcare_control navigation.launch.py map:=/path/care_map.yaml
    # 터미널 2
    ros2 launch smart_upcare_control demo_scenario.launch.py

    # 시뮬이면 use_sim_time:=true 통일
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='smart_upcare_control', executable='demo_scenario',
            name='demo_scenario_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
