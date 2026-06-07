import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lux = get_package_share_directory('lux')

    gim_params = os.path.join(pkg_lux, 'config', 'gim_params.yaml')
    joy_params = os.path.join(pkg_lux, 'config', 'joy_params.yaml')
    gim_calib  = os.path.join(pkg_lux, 'config', 'gim_servo_calib.yaml')

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

        # 2. 조이스틱 → Twist 변환
        Node(
            package='lux',
            executable='teleop_node_py',
            name='spot_teleop',
            output='screen',
            parameters=[joy_params]
        ),

        # 3. 상태 머신 (Twist → MiniCmd, 타임아웃/E-Stop 처리)
        Node(
            package='lux',
            executable='spot_sm_py',
            name='spot_sm',
            output='screen',
            parameters=[{'frequency': 200.0}]
        ),

        # 4. 기구학 & 보행 제어
        Node(
            package='lux',
            executable='spot_real_interface_ros2',
            name='spot_commander',
            output='screen',
            parameters=[gim_params, joy_params, gim_calib, {
                'use_imu': True,
                'dt': 0.01,
            }]
        ),

        # 5. MIT 모터 드라이버 (CAN)
        Node(
            package='lux',
            executable='mit_publisher_ros2',
            name='mit_publisher',
            output='screen',
            parameters=[
                gim_calib,
                {
                    'calib_yaml': gim_calib,
                    'can_interface': 'can0',
                    'bitrate': 500000,
                    'mit_kp': 60.0,
                    'mit_kd': 1.0,
                    'current_lim': 20.0,
                    'tx_rate_hz': 100.0,
                    'hold_while_wait': False,
                    'tx_on_no_cmd': True,
                    'cmd_timeout_sec': 0.3,
                    'center_on_start': False,
                    'offset_on_start': False,
                    'direct_after_startup': False,
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
