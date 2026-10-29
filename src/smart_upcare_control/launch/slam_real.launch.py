#!/usr/bin/env python3
"""
slam_real.launch.py — 실차 SLAM (slam_toolbox, 실물 UNO+라이다 기준)

  slam_4bed.launch.py 는 use_sim_time:='true' 가 하드코딩된 시뮬 전용
  파일이라 실차에서 쓸 수 없다(실물 SLAM 성공 레시피 0817·0829에서는
  slam_toolbox online_async_launch.py 를 인자 없이 직접 실행했는데,
  그러면 slam_params_file 이 안 넘어가 base_frame 이 기본값(base_footprint
  가 아닐 수 있음)으로 떠서 TF 불일치 위험이 있었다). 이 파일은 그 절차를
  slam_real.yaml(실차 전용 파라미터, use_sim_time:=false 고정)과 묶어
  재현 가능하게 만든 것.

  전제 (0829 인계 문서 §2 그대로):
    1) robot_bringup.launch.py 가 먼저 떠 있어야 함 (라이다+브리지+EKF+TF)
         ros2 launch smart_upcare_control robot_bringup.launch.py \
             uno_port:=/dev/ttyCH341USB0 use_virtual_lift:=true
       → "UNO 25Hz" + "ESP1 IMU ~50Hz" 로그 둘 다 확인. "IMU 수신 끊김" ERROR 가
         뜨면 매핑하지 말 것. (카메라 "No RealSense" 에러는 무시)
    2) ★ 10/4 갱신 — 회전(yaw)은 IMU 전담이다. ekf_real.yaml 은 9/5 에 휠 vyaw
       (odom0_config index 11)를 false 로 원복했으므로, IMU 없이(enable_esp1:=false)
       돌리면 EKF 에 회전 정보가 0 이 되어 맵이 부채꼴로 겹친다.
       (0829 의 enable_esp1:=false + 휠 vyaw=true 조합은 IMU 장착 전 임시 절차였음)

  실행:
    ros2 launch smart_upcare_control slam_real.launch.py

  주행 (별 터미널, SSH에서도 됨 — teleop xterm이 안 뜨므로 리맵 필수):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/cmd_vel_smoothed

  지도 저장 (★ map_subscribe_transient_local:=true 반드시 붙일 것 —
    안 붙이면 "Failed to spin map subscription" 로 저장 실패, 0817·0829 기록):
    ros2 run nav2_map_server map_saver_cli -f ~/hanium_ws/maps/my_map \
        --ros-args -p map_subscribe_transient_local:=true

  ※ online_async_launch.py 는 base_frame 을 launch 인자로 받지 않으므로
    slam_params_file 로 config/slam_real.yaml 을 넘긴다 (slam_4bed.launch.py 와
    동일 패턴 — 시뮬/실차는 slam_params_file 이 가리키는 파일로만 갈린다).
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
    pkg_slam = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(pkg_ctrl, 'config', 'slam_real.yaml')
    # 실차용 RViz 설정 파일은 아직 없음(§TODO) — 있으면 pkg_desc/rviz 쪽으로 지정.
    # 헤드리스(SSH) 운용이 기본이므로 rviz 기본값은 false.

    use_rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    args = [
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz 같이 띄울지 (실차는 보통 SSH 헤드리스라 기본 false)'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params,
                              description='slam_toolbox 파라미터 (base_frame 포함, 실차 전용)'),
        # [TODO] 실차용 rviz config 없음. 있으면 기본값 채울 것 — 지금은 rviz:=true 로
        #   띄우면 slam_toolbox 기본 rviz 설정으로 뜬다(빈 값이면 rviz2가 기본 레이아웃 사용).
        DeclareLaunchArgument('rviz_config', default_value=''),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',   # ★ 실차 고정. slam_4bed.launch.py의 'true'와 대칭.
            'slam_params_file': params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [slam, rviz])
