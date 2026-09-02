import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_upcare_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeongin',
    maintainer_email='yeongin@todo.todo',
    description='Smart Up-Care control nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 핵심 (B안): 브리지가 IK+FK+폐루프+odom 통합
            'serial_bridge = smart_upcare_control.serial_bridge_node:main',
            'lift_tf        = smart_upcare_control.lift_tf_node:main',
            'virtual_lift   = smart_upcare_control.virtual_lift_publisher:main',
            'demo_scenario  = smart_upcare_control.demo_scenario_node:main',
            # 실차 구동 검증(아두이노) — WASD 수동조종  ★ 8/7 구동팀본에서 병합
            'teleop_keyboard = smart_upcare_control.teleop_keyboard:main',
            # 진단용 유지
            'ekf_compare    = smart_upcare_control.ekf_compare:main',
            # (mecanum_ik / mecanum_odom 는 serial_bridge에 통합되어 제거됨)
        ],
    },
)
