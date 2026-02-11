from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('arm_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf.xacro')

    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
        Node(
	    package='ax12_hardware',
	    executable='ax12_joint_state_node',
	    output='screen'
	)
        
    ])

