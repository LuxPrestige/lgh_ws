from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	package_share = FindPackageShare('lux_description')
	use_gui = LaunchConfiguration('use_gui')

	xacro_file = PathJoinSubstitution([
		package_share,
		'urdf',
		'lux.urdf.xacro'
	])

	rviz_config_file = PathJoinSubstitution([
		package_share,
		'rviz',
		'lux.rviz'
	])

	robot_description = Command([
		FindExecutable(name='xacro'),
		' ',
		xacro_file
	])

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': robot_description
		}]
	)

	joint_state_publisher_gui_node = Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name='joint_state_publisher_gui',
		output='screen',
		condition=IfCondition(use_gui)
	)

	joint_state_bridge_node = Node(
		package='lux_description',
		executable='joint_state_bridge',
		name='lux_joint_state_bridge',
		output='screen',
		condition=UnlessCondition(use_gui),
		parameters=[{
			'source_topic': '/spot/joints',
			'degrees': True,
		}]
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file]
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_gui',
			default_value='false',
			description='Use joint_state_publisher_gui instead of live /spot/joints bridge'
		),
		robot_state_publisher_node,
		joint_state_publisher_gui_node,
		joint_state_bridge_node,
		rviz_node
	])
