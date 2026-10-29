#!/usr/bin/env python3
"""
slam_4bed.launch.py — hospital_4bed 월드용 SLAM (slam_toolbox + RViz)

  전제: 시뮬이 이미 떠 있어야 함
    ros2 launch smart_upcare_control gazebo_4bed.launch.py gui:=false lidar_rate:=5 imu_rate:=50
    ros2 service call /unpause_physics std_srvs/srv/Empty {}

  실행:
    ros2 launch smart_upcare_control slam_4bed.launch.py
    ros2 launch smart_upcare_control slam_4bed.launch.py rviz:=false   # RViz 없이

  운전 (별 터미널):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/cmd_vel_smoothed -p use_sim_time:=true

  지도 저장:
    ros2 run nav2_map_server map_saver_cli -f $HOME/hospital_4bed_map \
        --ros-args -p use_sim_time:=true

  ※ online_async_launch.py 는 base_frame 을 launch 인자로 받지 않으므로
    slam_params_file 로 config/slam_hospital_4bed.yaml 을 넘긴다.
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
    pkg_slam = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(pkg_ctrl, 'config', 'slam_hospital_4bed.yaml')
    default_rviz = os.path.join(pkg_desc, 'rviz', 'slam_4bed.rviz')

    use_rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    args = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz 같이 띄울지'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params,
                              description='slam_toolbox 파라미터 (base_frame 포함)'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [slam, rviz])
