import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 run lux keyboard_teleop_gim_py --ros-args --params-file ~/lux_ws/src/lux/config/gim_servo_calib.yaml
def generate_launch_description():
    pkg_lux = get_package_share_directory('lux')

    gim_params = os.path.join(pkg_lux, 'config', 'gim_params.yaml')
    joy_params = os.path.join(pkg_lux, 'config', 'joy_params.yaml')
    gim_calib  = os.path.join(pkg_lux, 'config', 'gim_servo_calib.yaml')

    return LaunchDescription([
        # 기구학 & 보행 제어
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

        # MIT 모터 드라이버 (CAN)
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
                    'mit_kd': 0.8,
                    'current_lim': 20.0,
                    'tx_rate_hz': 100.0,
                    'hold_while_wait': False,
                    'tx_on_no_cmd': True,
                    'cmd_timeout_sec': 0.3,
                    'center_on_start': False,
                    'offset_on_start': False,
                }
            ]
        ),

        # BNO085 IMU
        Node(
            package='lux',
            executable='bno085_node_py',
            name='bno085_node',
            output='screen'
        ),
    ])
