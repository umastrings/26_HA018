#!/usr/bin/env python3
"""
gazebo_4bed.launch.py — 4인실 시연 월드 원커맨드 브링업

  sim_bringup.launch.py 를 건드리지 않고, hospital_4bed.world + 간호스테이션
  스폰 좌표를 기본값으로 갖는 별도 진입점.
  기존 spawn.launch.py / ekf_sim.launch.py 를 그대로 include 하므로
  /scan, /odom, /odometry/filtered, /tf 파이프라인은 기존과 100% 동일.

  구성:
    - Gazebo (gzserver + gzclient) + hospital_4bed.world
    - 로봇 스폰 @ (-0.60, 0.00, yaw 0)  = 간호 스테이션, 복도 방향
    - robot_state_publisher (spawn.launch.py 내부)
    - EKF (ekf_sim.launch.py, use_sim_time=true)
    - lift_tf + virtual_lift (리프트 Dynamic TF)

  실행:
    ros2 launch smart_upcare_control gazebo_4bed.launch.py
    ros2 launch smart_upcare_control gazebo_4bed.launch.py gui:=false        # 헤드리스(VM)
    ros2 launch smart_upcare_control gazebo_4bed.launch.py spawn_x:=8.6 spawn_y:=0.0

  이후 (별 터미널):
    # SLAM 으로 지도 만들기
    ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=true slam_params_file:=<...> base_frame:=base_footprint
    ros2 run nav2_map_server map_saver_cli -f ~/hospital_4bed_map \
        --ros-args -p use_sim_time:=true

    # 저장된 지도로 Nav2
    ros2 launch smart_upcare_control navigation.launch.py \
        use_sim_time:=true map:=$HOME/hospital_4bed_map.yaml
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
    world_4bed = os.path.join(pkg_desc, 'worlds', 'hospital_4bed.world')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_virtual_lift = LaunchConfiguration('use_virtual_lift')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    lidar_samples = LaunchConfiguration('lidar_samples')
    lidar_rate = LaunchConfiguration('lidar_rate')
    imu_rate = LaunchConfiguration('imu_rate')
    camera_enabled = LaunchConfiguration('camera_enabled')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (VM 에서 FPS 낮으면 false)'),
        DeclareLaunchArgument('world', default_value=world_4bed,
                              description='기본: hospital_4bed.world'),
        DeclareLaunchArgument('use_virtual_lift', default_value='true',
                              description='리프트 실물 없이 lift_mm 모사'),
        # 간호 스테이션 스폰 — gen_hospital_4bed.py 검증 통과 좌표
        # 최근접 장애물 ns_chair 까지 0.381 m (inflation_radius 0.34 초과)
        DeclareLaunchArgument('spawn_x', default_value='-0.60'),
        DeclareLaunchArgument('spawn_y', default_value='0.00'),
        DeclareLaunchArgument('spawn_z', default_value='0.10',
                              description='스폰 높이. 롤오버 시 조정'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.00',
                              description='0 = +X(복도) 방향'),
        # VM 성능용 기본값. 정밀도가 필요하면 실행 시 올릴 것
        #   ros2 launch ... gazebo_4bed.launch.py lidar_samples:=1600 camera_enabled:=true
        DeclareLaunchArgument('lidar_samples', default_value='360'),
        DeclareLaunchArgument('lidar_rate', default_value='10'),
        DeclareLaunchArgument('imu_rate', default_value='100'),
        DeclareLaunchArgument('camera_enabled', default_value='false'),
    ]

    # ── Gazebo + 로봇 스폰 + robot_state_publisher ──
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'spawn.launch.py')),
        launch_arguments={
            'gui': gui,
            'world': world,
            'use_sim_time': 'true',
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
            'lidar_samples': lidar_samples,
            'lidar_rate': lidar_rate,
            'imu_rate': imu_rate,
            'camera_enabled': camera_enabled,
        }.items(),
    )

    # ── EKF (시뮬 config) ──
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
