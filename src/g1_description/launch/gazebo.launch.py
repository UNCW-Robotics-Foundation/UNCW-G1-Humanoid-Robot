from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('g1_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'g1_29dof_mode_15_brainco_hand.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'g1', '-z', '0.05'],
            output='screen'
        ),
    ])
