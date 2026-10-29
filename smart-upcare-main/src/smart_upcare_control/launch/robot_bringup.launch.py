#!/usr/bin/env python3
"""
robot_bringup.launch.py — 실물 로봇 브링업
  sensors + serial_bridge(3포트/폐루프) + EKF + robot_state_publisher + lift_tf

  실행:
    ros2 launch smart_upcare_control robot_bringup.launch.py
    # 리프트 실물 없이 가상 퍼블리셔로 Dynamic TF 검증:
    ros2 launch smart_upcare_control robot_bringup.launch.py use_virtual_lift:=true

  ★ 이 launch는 위치추정 백본까지만 올림. Nav2/SLAM은 navigation.launch.py로 분리.
  ★ 시뮬/실물 분기는 use_sim_time 로만. 실물 기본값 false.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ctrl = get_package_share_directory('smart_upcare_control')
    pkg_desc = get_package_share_directory('smart_upcare')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'smart_upcare.urdf.xacro')
    ekf_yaml = os.path.join(pkg_ctrl, 'config', 'ekf_real.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_virtual_lift = LaunchConfiguration('use_virtual_lift')
    enable_esp1 = LaunchConfiguration('enable_esp1')
    enable_esp2 = LaunchConfiguration('enable_esp2')
    track_width = LaunchConfiguration('track_width')
    wheelbase = LaunchConfiguration('wheelbase')
    bwd_gain = LaunchConfiguration('bwd_gain')
    uno_port = LaunchConfiguration('uno_port')
    esp1_port = LaunchConfiguration('esp1_port')
    esp2_port = LaunchConfiguration('esp2_port')
    lidar_port = LaunchConfiguration('lidar_port')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_virtual_lift', default_value='false',
                              description='리프트 실물 없이 lift_mm 모사 (§3)'),
        # ★ 8/7 추가: ESP32 미장착 상태에서 구동계 캘리브레이션을 하려면 필요
        DeclareLaunchArgument('enable_esp1', default_value='true',
                              description='false 면 IMU/lift 없이 UNO만 (캘리브레이션용)'),
        DeclareLaunchArgument('enable_esp2', default_value='false',
                              description='ESP32 2대 확보 시 true (§6)'),
        # ★ 9/5 — 이 launch 인자 기본값이 소스 기본값을 항상 덮어써서 track_width 수정이
        #   반영 안 되는 버그의 실제 원인이었다 (CLAUDE.md 참조). 0.20699는 URDF 실측치일
        #   뿐, 90도 회전 odom 재보정 결과인 0.1354와 달라 여기 값만 고쳐도 되돌아간다.
        DeclareLaunchArgument('track_width', default_value='0.1354'),

        DeclareLaunchArgument('wheelbase', default_value='0.19205'),    # STEP 실측 7/29
        # ★ 10/4 추가 — 직진 비대칭 보정값. track_width 때 겪은 "launch 기본값이
        #   소스 기본값을 덮어쓰는" 사고를 반복하지 않으려고 노드 기본값과 항상
        #   똑같이 맞춰둘 것. 같은 날 1m 전진/후진 반복 측정으로 1.25->1.0625
        #   재튜닝 확정 (CLAUDE.md §10/4 bwd_gain 재튜닝 참조). 재주행 튜닝은
        #   재시작 없이 `ros2 param set /serial_bridge_node bwd_gain <값>`으로.
        DeclareLaunchArgument('bwd_gain', default_value='1.0625'),
        # ─── ★ 8/7 추가: 포트를 전부 launch 인자로 노출 ───
        #   [배경] 기존 기본값이 lidar_port == esp1_port == /dev/ttyUSB0 로 충돌해
        #     robot_bringup 이 둘을 동시에 올리면 나중 쪽이 포트를 못 열었다.
        #   [운용] ttyUSB 번호는 꽂는 순서로 바뀌므로 기본값 변경으로는 못 막는다.
        #     젯슨에서 `ls -l /dev/serial/by-id/` 로 확인한 고정 경로를 주입할 것:
        #       ros2 launch smart_upcare_control robot_bringup.launch.py \
        #           uno_port:=/dev/serial/by-id/usb-...-if00 \
        #           lidar_port:=/dev/serial/by-id/usb-...-if00-port0
        # ★ 8/7 구동팀 회신 §6-1: 실차 UNO 는 CH340 클론 → ttyCH341USB0
        DeclareLaunchArgument('uno_port', default_value='/dev/ttyCH341USB0'),
        # ★ 10/4 — 실차 ESP1(ESP32-S3)은 ttyACM0 (9/5 실측). 기존 ttyUSB1 기본값 때문에
        #   인자를 빼먹으면 ESP1 을 못 열고 IMU 없이 돌았다. 이제 기본값으로 맞고,
        #   그래도 못 열면 브리지가 기동 실패한다(IMU 없이 띄우려면 enable_esp1:=false).
        DeclareLaunchArgument('esp1_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('esp2_port', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
    ]

    # ★ 8/7 — 8/4 교훈(navigation.launch.py §40-44) backport.
    #   dict 에 LaunchConfiguration 을 그냥 넣으면 문자열로 전달되어 노드가
    #   무시한다. use_sim_time 뿐 아니라 bool/double 파라미터 전부 해당.
    #   특히 track_width/wheelbase 는 무시돼도 에러가 없어 IK 상수가 조용히
    #   기본값으로 남는다 — 조립 후 실측값 주입이 무효가 되는 경로.
    sim_time_bool = ParameterValue(use_sim_time, value_type=bool)
    esp1_bool = ParameterValue(enable_esp1, value_type=bool)
    esp2_bool = ParameterValue(enable_esp2, value_type=bool)
    track_width_f = ParameterValue(track_width, value_type=float)
    wheelbase_f = ParameterValue(wheelbase, value_type=float)
    bwd_gain_f = ParameterValue(bwd_gain, value_type=float)

    # ── URDF (실측 인자 주입 — IK/EKF와 동일 소스) ──
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' track_width:=', track_width,
                 ' wheelbase:=', wheelbase]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': sim_time_bool}],
    )

    # ── 센서 드라이버 ──
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, 'launch', 'sensors.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'lidar_port': lidar_port}.items(),
    )

    # ── 3포트 시리얼 브리지 + 폐루프 ──
    serial_bridge = Node(
        package='smart_upcare_control', executable='serial_bridge',
        name='serial_bridge_node', output='screen',
        parameters=[{
            'use_sim_time': sim_time_bool,
            'enable_esp1': esp1_bool,
            'enable_esp2': esp2_bool,
            'track_width': track_width_f,
            'wheelbase': wheelbase_f,
            'bwd_gain': bwd_gain_f,
            'uno_port': uno_port,
            'esp1_port': esp1_port,
            'esp2_port': esp2_port,
            # 나머지(게인/ff_gain/wheel_sign)는 노드 기본값 → 실측 후 조정
        }],
    )

    # ── EKF (odom->base_footprint TF 소유) ──
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': sim_time_bool}],
    )

    # ── 리프트 Dynamic TF ──
    lift_tf = Node(
        package='smart_upcare_control', executable='lift_tf',
        name='lift_tf_node', output='screen',
        parameters=[{'use_sim_time': sim_time_bool}],
    )
    virtual_lift = Node(
        package='smart_upcare_control', executable='virtual_lift',
        name='virtual_lift_publisher', output='screen',
        condition=IfCondition(use_virtual_lift),
        parameters=[{'use_sim_time': sim_time_bool}],
    )

    return LaunchDescription(
        args + [rsp, sensors, serial_bridge, ekf, lift_tf, virtual_lift])
