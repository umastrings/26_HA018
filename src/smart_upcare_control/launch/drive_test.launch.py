#!/usr/bin/env python3
"""
drive_test.launch.py — 아두이노 구동부만 단독 검증 (실차 통합 1단계)

  라이다·카메라·Nav2·리프트 없이 **UNO + 모터/엔코더**만 올려서
  "젯슨 ↔ 아두이노 통신 + 바퀴 구동 + 오도메트리"를 확인하는 런치.

  실행:
    ros2 launch smart_upcare_control drive_test.launch.py
    ros2 launch smart_upcare_control drive_test.launch.py uno_port:=/dev/ttyACM0
    ros2 launch smart_upcare_control drive_test.launch.py teleop:=false   # Nav2 등 외부 명령 테스트

  ★ 기본 포트가 /dev/ttyCH341USB0 인 이유:
    실차 UNO 는 CH340 클론이라 ttyACM0/ttyUSB0 로 안 잡힘. WCH ch341ser 드라이버 필요.
    (정품 UNO 로 교체 시 uno_port:=/dev/ttyACM0)

  ★ ESP32(IMU/리프트) 미연결 상태를 가정 → enable_esp1:=false 로 브리지가 UNO 만 연다.
    IMU 가 없으므로 EKF 는 띄우지 않는다. 여기서 보는 오도메트리는 /odom_raw (엔코더 기반).

  ★ teleop 은 /cmd_vel 로 발행 → 브리지가 구독하는 /cmd_vel_smoothed 로 remap.
    (실운용에서는 Nav2 → collision_monitor → /cmd_vel_smoothed 경로를 탄다)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    uno_port = LaunchConfiguration('uno_port')
    teleop = LaunchConfiguration('teleop')
    track_width = LaunchConfiguration('track_width')
    wheelbase = LaunchConfiguration('wheelbase')

    args = [
        DeclareLaunchArgument('uno_port', default_value='/dev/ttyCH341USB0',
                              description='UNO 시리얼 포트 (CH340 클론 기준)'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='WASD 조종 창(xterm) 띄우기'),
        # ★ 9/1 정정 — 0.20699(STEP 실측)는 90도 회전 테스트에서 odom이 실제
        #   회전각을 과소평가(60~65도)하는 것으로 확인돼 0.1354로 재보정.
        DeclareLaunchArgument('track_width', default_value='0.1354'),
        DeclareLaunchArgument('wheelbase', default_value='0.19205'),
    ]

    # ★ 8/7 병합 시 추가 — LaunchConfiguration 을 dict 에 그냥 넣으면 문자열로
    #   전달되어 노드가 무시한다 (8/4 교훈, navigation.launch.py §40-44).
    #   track_width/wheelbase 는 무시돼도 에러가 없어서 조용히 기본값이 남는다.
    track_width_f = ParameterValue(track_width, value_type=float)
    wheelbase_f = ParameterValue(wheelbase, value_type=float)

    # 시리얼 브리지 — UNO 단독 (IK/FK/폐루프/odom 포함)
    bridge = Node(
        package='smart_upcare_control', executable='serial_bridge',
        name='serial_bridge_node', output='screen',
        parameters=[{
            'uno_port': uno_port,
            'enable_esp1': False,     # ESP32#1(IMU/리프트) 미연결
            'enable_esp2': False,
            'track_width': track_width_f,
            'wheelbase': wheelbase_f,
        }],
    )

    # WASD 수동조종 — /cmd_vel → /cmd_vel_smoothed 로 직결
    teleop_node = Node(
        package='smart_upcare_control', executable='teleop_keyboard',
        name='teleop_keyboard', output='screen',
        prefix='xterm -e', condition=IfCondition(teleop),
        remappings=[('cmd_vel', 'cmd_vel_smoothed')],
    )

    return LaunchDescription(args + [bridge, teleop_node])
