"""
ekf_test.launch.py — EKF 검증용 런치

  포함:
    - ekf_filter_node        (robot_localization)
    - static TF: base_link -> imu_link      ★ EKF 가 필요로 함
    - static TF: base_link -> lidar_link    (나중 SLAM 대비)

  ★ 주의: 이건 "검증용" 런치다.
    실물에서는 URDF + robot_state_publisher 가 TF 를 발행하므로
    이 static TF 들은 빼야 한다 (중복 발행 = TF 충돌).

사용법:
  ros2 launch smart_upcare_control ekf_test.launch.py

  (선행: socat + fake_uno + serial_bridge 실행중이어야 함. serial_bridge가 /odom_raw 발행)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('smart_upcare_control')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf_mecanum.yaml')

    return LaunchDescription([
        # ---- EKF ----
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml],
        ),

        # ---- IMU TF (EKF 필수) ----
        # BNO055 가 base_link 중심에서 z=+0.05m 위에 있다고 가정
        # ★ 실물 장착 위치로 교체할 것
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        # ---- LiDAR TF (SLAM 대비) ----
        # ★ 실물 장착 위치로 교체할 것
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0.05', '0', '0.12', '0', '0', '0',
                       'base_link', 'lidar_link'],
        ),
    ])
