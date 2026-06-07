import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 run lux keyboard_teleop_spot_py --ros-args --params-file ~/lux_ws/src/lux/config/spot_servo_calib.yaml
def generate_launch_description():
    pkg_lux = get_package_share_directory('lux')

    spot_params = os.path.join(pkg_lux, 'config', 'spot_params.yaml')
    joy_params = os.path.join(pkg_lux, 'config', 'joy_params.yaml')
    spot_calib  = os.path.join(pkg_lux, 'config', 'spot_servo_calib.yaml')

    return LaunchDescription([
        # 기구학 & 보행 제어
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

        # MIT 모터 드라이버 (CAN)
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

        # BNO085 IMU
        Node(
            package='lux',
            executable='bno085_node_py',
            name='bno085_node',
            output='screen'
        ),
    ])
