"""ekf_sim.launch.py — Gazebo 시뮬용 EKF (static TF 없음: URDF가 발행)"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('smart_upcare_control')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf_mecanum.yaml')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': True}],
        ),
    ])
