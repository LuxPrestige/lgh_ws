from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lux'

setup(
    name=package_name,
    version='0.0.0',
    # 패키지 내의 하위 폴더(Kinematics, GaitGenerator)를 인식하기 위해 find_packages 사용
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치 경로 설정
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # 설정 파일(YAML) 설치 경로 설정
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Lux Robot Python Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # [수정 핵심] '실행파일명 = 패키지명.파일명:함수명'
            'spot_real_interface_ros2 = lux.spot_real_interface_ros2:main',
            'mit_publisher_ros2 = lux.mit_publisher_ros2:main',
            'spot_sm_py = lux.spot_sm:main',
            'teleop_node_py = lux.teleop_node:main',
            'dashboard_py = lux.dashboard:main',
            'pygame_visualizer = lux.pygame_visualizer:main',
            'motor_calibrator_ros2 = lux.motor_calibrator_ros2:main',
            'lux_pygame_dashboard = lux.lux_pygame_dashboard:main',
            'bno085_node_py = lux.bno085_node:main',
            'pwm_publisher_ros2 = lux.pwm_publisher:main',
            'keyboard_teleop_py = lux.keyboard_teleop:main',
            'keyboard_teleop_gim_py = lux.keyboard_teleop_gim:main',
            'keyboard_teleop_spot_py = lux.keyboard_teleop_spot:main',
        ],
    },
)
