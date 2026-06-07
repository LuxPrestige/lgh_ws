import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lux = get_package_share_directory('lux')

    spot_params   = os.path.join(pkg_lux, 'config', 'spot_params.yaml')
    joy_params    = os.path.join(pkg_lux, 'config', 'joy_params.yaml')
    spot_calib    = os.path.join(pkg_lux, 'config', 'spot_servo_calib.yaml')

    return LaunchDescription([

        # 1. 조이스틱 드라이버
        Node(
            package='joy',
            executable='joy_node',
            name='spot_joy',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 2. 텔레오퍼레이션
        Node(
            package='lux',
            executable='teleop_node_py',
            name='spot_teleop',
            output='screen',
            parameters=[joy_params]
        ),

        # 3. 상태 머신
        Node(
            package='lux',
            executable='spot_sm_py',
            name='spot_sm',
            output='screen',
            parameters=[{'frequency': 200.0}]
        ),

        # 4. 기구학 & 보행 제어 (캘리브레이션 포함)
        Node(
            package='lux',
            executable='spot_real_interface_ros2',
            name='spot_commander',
            output='screen',
            parameters=[spot_params, joy_params, spot_calib, {
                'use_imu': True,
                'dt': 0.01,
            }]
        ),

        # 5. PWM 서보 드라이버 (PCA9685 I2C)
        Node(
            package='lux',
            executable='pwm_publisher_ros2',
            name='pwm_publisher',
            output='screen',
            parameters=[
                spot_calib,
                {
                    'freq': 50,
                    'pulse_min': 500,
                    'pulse_max': 2500,
                    'max_vel_bypass': 5.0,
                }
            ]
        ),

        # 6. BNO085 IMU
        Node(
            package='lux',
            executable='bno085_node_py',
            name='bno085_node',
            output='screen'
        ),
    ])
