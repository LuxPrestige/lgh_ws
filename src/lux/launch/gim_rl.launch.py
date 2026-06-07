import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_lux = get_package_share_directory("lux")
    gim_calib = os.path.join(pkg_lux, "config", "gim_servo_calib.yaml")
    gim_params = os.path.join(pkg_lux, "config", "gim_params.yaml")

    pkg_lux_description = FindPackageShare("lux_description")
    xacro_file = PathJoinSubstitution([pkg_lux_description, "urdf", "lux.urdf.xacro"])
    rviz_config_file = PathJoinSubstitution([pkg_lux_description, "rviz", "lux.rviz"])
    robot_description = Command(["xacro ", xacro_file])

    default_policy_path = PathJoinSubstitution(
        [FindPackageShare("lux"), "policies", LaunchConfiguration("policy_file")]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_file",
                default_value="policy.pt",
                description="TorchScript policy filename under share/lux/policies",
            ),
            DeclareLaunchArgument(
                "policy_path",
                default_value=default_policy_path,
                description="TorchScript policy path. Overrides policy_file when set.",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="false",
                description="Start robot_state_publisher, joint_state_bridge, and RViz2",
            ),
            DeclareLaunchArgument(
                "joint_state_source",
                default_value="/spot/joints",
                description="JointAngles topic used by lux_description joint_state_bridge",
            ),
            DeclareLaunchArgument(
                "use_imu_node",
                default_value="false",
                description="Start legacy BNO085 IMU topic node",
            ),
            DeclareLaunchArgument(
                "use_direct_bno085",
                default_value="true",
                description="Read BNO085 accel, gyro, and quaternion directly in lux_rl_interface",
            ),
            Node(
                package="lux",
                executable="lux_rl_interface",
                name="lux_rl_interface",
                output="screen",
                parameters=[
                    gim_params,
                    gim_calib,
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "calib_yaml": gim_calib,
                        "cmd_vx_scale": 1.0,
                        "cmd_vy_scale": 1.0,
                        "cmd_wz_scale": 1.0,
                        "use_direct_bno085": LaunchConfiguration("use_direct_bno085"),
                        "bno085_rate_hz": 100.0,
                    },
                ],
            ),
            Node(
                package="lux",
                executable="mit_publisher_ros2",
                name="mit_publisher",
                output="screen",
                parameters=[
                    gim_calib,
                    {
                        "calib_yaml": gim_calib,
                        "can_interface": "can0",
                        "bitrate": 500000,
                        "mit_kp": 45.0,
                        "mit_kd": 1.0,
                        "current_lim": 20.0,
                        "tx_rate_hz": 100.0,
                        "hold_while_wait": False,
                        "tx_on_no_cmd": True,
                        "cmd_timeout_sec": 0.3,
                        "direct_after_startup": False,
                        "center_on_start": False,
                        "offset_on_start": False,
                    },
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("visualize")),
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="lux_description",
                executable="joint_state_bridge",
                name="lux_joint_state_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("visualize")),
                parameters=[
                    {
                        "source_topic": LaunchConfiguration("joint_state_source"),
                        "degrees": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("visualize")),
                arguments=["-d", rviz_config_file],
            ),
            Node(
                package="lux",
                executable="bno085_node_py",
                name="bno085_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_imu_node")),
            ),
        ]
    )
