#!/usr/bin/env python3
"""
sim_bringup.launch.py — Gazebo 시뮬 원커맨드 브링업 (실물 robot_bringup의 시뮬 짝)

  실물의 serial_bridge/sensors 자리를 Gazebo가 대체:
    - planar_move   → /odom_raw (구동+오도메트리)
    - gazebo 센서    → /scan, /imu/data, camera
  공용 노드는 그대로, use_sim_time:=true 로 통일:
    - EKF (ekf_sim: ekf_mecanum.yaml)
    - robot_state_publisher (spawn.launch 내부)
    - lift_tf + virtual_lift (리프트 실물 없이 Dynamic TF)

  실행:
    ros2 launch smart_upcare_control sim_bringup.launch.py            # GUI
    ros2 launch smart_upcare_control sim_bringup.launch.py gui:=false # 헤드리스(VM 권장)

  이후:
    ros2 launch smart_upcare_control navigation.launch.py use_sim_time:=true map:=~/care_map.yaml
    ros2 launch smart_upcare_control demo_scenario.launch.py use_sim_time:=true
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
    pkg_ctrl = get_package_share_directory('smart_upcare_control')
    pkg_desc = get_package_share_directory('smart_upcare')
    default_world = os.path.join(pkg_desc, 'worlds', 'care_facility.world')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_virtual_lift = LaunchConfiguration('use_virtual_lift')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (VM 느리면 false)'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='케어시설 월드 (기본: care_facility.world)'),
        DeclareLaunchArgument('use_virtual_lift', default_value='true',
                              description='리프트 실물 없이 lift_mm 모사 (시뮬 기본 on)'),
    ]

    # ── Gazebo + 로봇 스폰 + robot_state_publisher (use_sim_time=true 기본) ──
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'spawn.launch.py')),
        launch_arguments={
            'gui': gui,
            'world': world,
            'use_sim_time': 'true',
        }.items(),
    )

    # ── EKF (시뮬 config, use_sim_time=true 내장) ──
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, 'launch', 'ekf_sim.launch.py')),
    )

    # ── 리프트 Dynamic TF ──
    lift_tf = Node(
        package='smart_upcare_control', executable='lift_tf',
        name='lift_tf_node', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    virtual_lift = Node(
        package='smart_upcare_control', executable='virtual_lift',
        name='virtual_lift_publisher', output='screen',
        condition=IfCondition(use_virtual_lift),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [spawn, ekf, lift_tf, virtual_lift])
