#!/usr/bin/env python3
# spawn.launch.py
# Gazebo 실행 + 로봇 스폰 (+ robot_state_publisher)
# 실행 예:
#   ros2 launch smart_upcare spawn.launch.py                 (GUI)
#   ros2 launch smart_upcare spawn.launch.py gui:=false      (헤드리스, VM 권장)
#   ros2 launch smart_upcare spawn.launch.py world:=/path/to.world

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('smart_upcare')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(pkg, 'urdf', 'smart_upcare.urdf.xacro')

    # ── launch 인자 ──
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    track_width = LaunchConfiguration('track_width')
    wheelbase = LaunchConfiguration('wheelbase')
    # ── 시뮬 센서 부하 (7/31 추가) — URDF <gazebo> 블록에만 영향, 실물 무관 ──
    lidar_samples = LaunchConfiguration('lidar_samples')
    lidar_rate = LaunchConfiguration('lidar_rate')
    imu_rate = LaunchConfiguration('imu_rate')
    camera_enabled = LaunchConfiguration('camera_enabled')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Gazebo GUI 띄울지. VM에서 느리면 false (헤드리스)')
    declare_world = DeclareLaunchArgument(
        'world', default_value='',
        description='.world 파일 경로 (비우면 빈 월드)')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='시뮬 시간 사용 (SLAM/Nav2와 반드시 통일)')
    # ── 스폰 좌표 (기존 line82 하드코딩 → arg화) ──
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-3.5')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-0.3')
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='0.1',
        description='스폰 높이. 롤오버 시 조정(§3)')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    # ── 실측 오버라이드 (URDF xacro:arg로 전달 → 기구학·footprint 정합) ──
    declare_track = DeclareLaunchArgument(
        'track_width', default_value='0.20699', description='STEP 실측 7/29')
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase', default_value='0.19205', description='STEP 실측 7/29')
    # ── 시뮬 성능 조절 (VM llvmpipe 대응). URDF 기본값과 동일하게 맞춰둠 ──
    declare_lidar_samples = DeclareLaunchArgument(
        'lidar_samples', default_value='360',
        description='시뮬 LiDAR 샘플 수. 실물 A2M12 와 무관. 정밀도 필요시 1600')
    declare_lidar_rate = DeclareLaunchArgument(
        'lidar_rate', default_value='10',
        description='시뮬 LiDAR Hz. CPU 부족으로 스톨 생기면 5로 낮출 것')
    declare_imu_rate = DeclareLaunchArgument(
        'imu_rate', default_value='100',
        description='시뮬 IMU Hz. EKF가 update rate 못 맞추면 50으로')
    declare_camera_enabled = DeclareLaunchArgument(
        'camera_enabled', default_value='false',
        description='시뮬 뎁스카메라 on/off. VM에서 FPS 최대 소모원. 연구트랙은 true')

    # ── xacro → URDF (실측 인자 주입) ──
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' track_width:=', track_width,
                 ' wheelbase:=', wheelbase,
                 ' lidar_samples:=', lidar_samples,
                 ' lidar_rate:=', lidar_rate,
                 ' imu_rate:=', imu_rate,
                 ' camera_enabled:=', camera_enabled]),
        value_type=str)

    # ── robot_state_publisher (TF 발행) ──
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Gazebo 서버 (gzserver) — 물리 담당 ──
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            # 아래 두 개가 있어야 /scan, /odom 등 ROS 토픽이 나옴
            'init': 'true', 'factory': 'true',
        }.items(),
    )

    # ── Gazebo 클라이언트 (gzclient) — GUI. gui:=false면 안 뜸 ──
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # ── 로봇을 Gazebo에 스폰 ──
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'smart_upcare',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    return LaunchDescription([
        declare_gui, declare_world, declare_sim_time,
        declare_spawn_x, declare_spawn_y, declare_spawn_z, declare_spawn_yaw,
        declare_track, declare_wheelbase,
        declare_lidar_samples, declare_lidar_rate, declare_imu_rate,
        declare_camera_enabled,
        rsp, gzserver, gzclient, spawn,
    ])
