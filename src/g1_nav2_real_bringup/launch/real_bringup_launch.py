import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # package directories
    pkg_g1_description  = get_package_share_directory('g1_description')
    pkg_real_bringup    = get_package_share_directory('g1_nav2_real_bringup')
    pkg_nav2_bringup    = get_package_share_directory('nav2_bringup')
    pkg_sim_bringup     = get_package_share_directory('g1_nav2_sim_bringup')

    # URDF
    urdf_file = os.path.join(pkg_g1_description, 'urdf', 'g1_29dof_mode_15_brainco_hand.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # config files
    fast_lio_config = os.path.join(pkg_real_bringup, 'config', 'fast_lio_mid360.yaml')
    slam_params     = os.path.join(pkg_real_bringup, 'config', 'slam_toolbox_params.yaml')
    nav2_params     = os.path.join(pkg_real_bringup, 'config', 'nav2_params.yaml')
    rviz_config     = os.path.join(pkg_sim_bringup,  'config', 'g1_bringup.rviz')

    # launch arguments
    rviz_use = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # reads real motor states from lf/lowstate, then publishes /joint_states
    lowstate_jointstate_bridge_node = Node(
        package='g1_lower',
        executable='lowstate_jointstate_bridge',
        output='screen'
    )

    # flips the head imu to match the orientation of the robot (/utlidar/imu_livox_mid360 -> /imu/upright)
    imu_frameflip_node = Node(
        package='g1_odom_tools',
        executable='imu_frameflip',
        output='screen'
    )

    # static odom to lio_odom anchor that puts that plane at the floor (z = 0)
    floor_level_anchor_node = Node(
        package='g1_odom_tools',
        executable='floor_level_anchor',
        output='screen'
    )

    lio_to_base_broadcaster_node = Node(
        package='g1_odom_tools',
        executable='lio_to_base_broadcaster',
        output='screen'
    )

    # bridges livox_frame to mid360_link, must be level   
    static_tf_livox_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '3.14159', 'mid360_link', 'livox_frame']
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[fast_lio_config],
        output='screen'
    )

    # flattens 3d pointcloud to 2d scan for nav2 costmaps.
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/utlidar/cloud_livox_mid360')],
        parameters=[{'target_frame': 'base_link',
                     'min_height': 0.3,
                     'max_height': 1.6,
                     # ignore shoulders of g1
                     'range_min': 0.7,
                     
                     'range_max': 20.0}]
    )


    # builds 2d occupancy map from /scan, publishes map to odom tf and /map
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_params],
        output='log'
    )


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false'
        }.items()
    )


    # translates nav2 velocity commands (/cmd_vel) into unitree movement (api/sport/request)
    nav2_cmd_controller_node = Node(
        package='g1_nav2_real_bringup',
        executable='nav2_cmd_controller',
        output='screen'
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_use),
        output='log'
    )


    ld = LaunchDescription()

    ld.add_action(declare_rviz_cmd)

    # runs on start
    ld.add_action(robot_state_publisher_node)
    ld.add_action(lowstate_jointstate_bridge_node)
    ld.add_action(static_tf_livox_node)
    ld.add_action(imu_frameflip_node)
    ld.add_action(floor_level_anchor_node)
    ld.add_action(lio_to_base_broadcaster_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(nav2_cmd_controller_node)
    ld.add_action(rviz_node)
    


    # delay fast-lio 5s,  needs TF tree first
    ld.add_action(TimerAction(period=5.0, actions=[fast_lio_node])) # period=5.0,

    # delay slam and nav2, needs fast-lio odometry to finish initializing before maps generate
    ld.add_action(TimerAction(period=25.0, actions=[slam_node]))
    ld.add_action(TimerAction(period=25.0, actions=[nav2_launch]))

    return ld
