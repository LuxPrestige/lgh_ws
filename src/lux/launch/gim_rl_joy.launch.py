from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gim_rl_launch = PathJoinSubstitution(
        [FindPackageShare("lux"), "launch", "gim_rl.launch.py"]
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
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lux"), "policies", LaunchConfiguration("policy_file")]
                ),
                description="TorchScript policy path. Overrides policy_file when set.",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="false",
                description="Start robot_state_publisher, joint_state_bridge, and RViz2",
            ),
            DeclareLaunchArgument(
                "use_direct_bno085",
                default_value="true",
                description="Read BNO085 directly in lux_rl_interface",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gim_rl_launch),
                launch_arguments={
                    "policy_file": LaunchConfiguration("policy_file"),
                    "policy_path": LaunchConfiguration("policy_path"),
                    "visualize": LaunchConfiguration("visualize"),
                    "use_direct_bno085": LaunchConfiguration("use_direct_bno085"),
                }.items(),
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="spot_joy",
                parameters=[
                    {
                        "dev": "/dev/input/js0",
                        "deadzone": 0.05,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="lux",
                executable="teleop_node_py",
                name="rl_teleop",
                output="screen",
                remappings=[("teleop", "/cmd_vel")],
            ),
        ]
    )
