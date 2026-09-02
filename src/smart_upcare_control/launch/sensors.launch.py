#!/usr/bin/env python3
"""
sensors.launch.py — rplidar + realsense + depthimage_to_laserscan (실물 전용)

  시뮬에선 Gazebo 플러그인이 /scan, /camera/* 를 직접 냄 → 이 launch 미사용.
  실물에서만 실제 드라이버를 띄운다.

  실행:
    ros2 launch smart_upcare_control sensors.launch.py
    ros2 launch smart_upcare_control sensors.launch.py enable_depth_scan:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_depth_scan = LaunchConfiguration('enable_depth_scan')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # ★ 8/7 — 기본값이 serial_bridge 의 esp1_port(/dev/ttyUSB0)와 겹쳐서
        #   robot_bringup 이 둘을 동시에 올리면 나중 쪽이 포트를 못 열었다.
        #   기본값을 분리했지만, ttyUSB 번호는 꽂는 순서로 바뀌므로 근본 해결은
        #   `/dev/serial/by-id/` 고정 경로 주입이다 (§2-1).
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('enable_depth_scan', default_value='true',
                              description='D415 depth → LaserScan (Nav2 costmap 투입)'),
    ]

    # ★ 8/7 — dict 에 LaunchConfiguration 을 그냥 넣으면 문자열로 전달되어
    #   노드가 무시한다 (8/4 교훈, navigation.launch.py §40-44).
    sim_time_bool = ParameterValue(use_sim_time, value_type=bool)

    # ── RPLIDAR A2M12 ──
    rplidar = Node(
        package='rplidar_ros', executable='rplidar_composition',
        name='rplidar', output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 256000,       # A2M12
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': sim_time_bool,
        }],
    )

    # ── RealSense D415 (USB2 고착 대응 저해상도 — 한이음 트랙, memory) ──
    realsense = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        name='camera', output='screen',
        parameters=[{
            'depth_module.profile': '480x270x6',   # USB2 대역 내
            'rgb_camera.profile': '424x240x6',
            'enable_infra1': False,                 # IR 비활성 (USB2)
            'enable_infra2': False,
            'pointcloud.enable': False,
            'initial_reset': True,
            'use_sim_time': sim_time_bool,
        }],
    )

    # ── depth → LaserScan (costmap용 저상 장애물) ──
    depth_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan', output='screen',
        condition=IfCondition(enable_depth_scan),
        remappings=[
            ('depth', '/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_depth'),      # /scan(라이다)과 분리, costmap에서 별도 소스로
        ],
        parameters=[{
            'output_frame': 'camera_optical_link',
            'range_min': 0.2,
            'range_max': 10.0,
            'use_sim_time': sim_time_bool,
        }],
    )

    return LaunchDescription(args + [rplidar, realsense, depth_scan])
