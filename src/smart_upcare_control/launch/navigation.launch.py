#!/usr/bin/env python3
"""
navigation.launch.py — Nav2 + Collision Monitor,  slam:=true/false 분기

  slam:=true   SLAM(slam_toolbox)로 맵 생성하며 주행 (초기 맵 만들 때)
  slam:=false  저장된 맵 + AMCL 로 위치추정 주행 (기본, 데모)

  실행:
    ros2 launch smart_upcare_control navigation.launch.py                 # AMCL
    ros2 launch smart_upcare_control navigation.launch.py slam:=true      # SLAM
    ros2 launch smart_upcare_control navigation.launch.py use_sim_time:=true map:=/home/.../care_map.yaml

  ★ 순서(§3): Collision Monitor를 여기서 함께 올림. cmd_vel(컨트롤러)
    → collision_monitor → cmd_vel_smoothed(→ serial_bridge). 튜닝은 이 체인 붙인 뒤.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ctrl = get_package_share_directory('smart_upcare_control')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_ctrl, 'config', 'nav2_care.yaml')
    collision_yaml = os.path.join(pkg_ctrl, 'config', 'collision_monitor.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')

    # ★ 8/4 — use_sim_time 을 문자열이 아니라 bool 로 확정해 전달.
    #   dict 로 LaunchConfiguration 을 그냥 넘기면 문자열 'true' 로 전달되어
    #   노드가 무시하고 벽시계를 쓰는 경우가 있다. collision_monitor 로그의
    #   "timestamps differ on 1785821095 seconds" 가 그 증상이었다.
    sim_time_bool = ParameterValue(use_sim_time, value_type=bool)

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true=slam_toolbox, false=map+AMCL'),
        DeclareLaunchArgument('map', default_value='',
                              description='.yaml 맵 경로 (slam:=false일 때 필수)'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # ★ 8/3 추가: Collision Monitor on/off
        #   false 로 두면 controller 의 /cmd_vel 을 relay 로 /cmd_vel_smoothed 에
        #   그대로 흘려보낸다. Nav2 자체 문제와 Collision Monitor 문제를
        #   분리해서 볼 때 사용.
        DeclareLaunchArgument('use_collision_monitor', default_value='true',
                              description='false 면 relay 로 우회 (문제 분리용)'),
    ]

    # ── ★ 8/3 수정: slam 인자 대소문자 정규화 ──
    # nav2_bringup/bringup_launch.py 내부에
    #     condition=IfCondition(PythonExpression(['not ', slam]))
    # 가 있다. 문자열을 이어붙여 파이썬으로 평가하므로 소문자 'false' 를 넘기면
    #     not false  ->  NameError: name 'false' is not defined
    # 로 launch 전체가 죽는다. 반드시 'True' / 'False' 여야 한다.
    # 사용자가 true/True/false/False 중 뭘 쓰든 받도록 여기서 변환한다.
    slam_norm = PythonExpression(
        ["'True' if '", slam, "'.lower() in ('true','1') else 'False'"])

    # ── Nav2 코어 (nav2_bringup 재사용: slam/map/amcl/lifecycle 일괄) ──
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam_norm,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    # ── Collision Monitor + 전용 lifecycle 관리자 ──
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[collision_yaml, {'use_sim_time': sim_time_bool}],
        condition=IfCondition(use_collision_monitor),
    )
    lifecycle_collision = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_collision', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['collision_monitor'],
        }],
        condition=IfCondition(use_collision_monitor),
    )

    # ── 우회 relay (use_collision_monitor:=false 일 때만) ──
    #   planar_move / serial_bridge 는 /cmd_vel_smoothed 를 구독하므로
    #   Collision Monitor 를 빼면 중간이 끊긴다. relay 로 이어준다.
    #   topic_tools 가 없으면: sudo apt install ros-humble-topic-tools
    cmd_vel_relay = Node(
        package='topic_tools', executable='relay',
        name='cmd_vel_relay', output='screen',
        arguments=['/cmd_vel', '/cmd_vel_smoothed'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_collision_monitor),
    )

    return LaunchDescription(
        args + [nav2, collision_monitor, lifecycle_collision, cmd_vel_relay])
