import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


from launch_ros.actions import Node


   
        
 
def generate_launch_description():
    #urdf description of g1
    pkg_share = get_package_share_directory('g1_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'g1_29dof_mode_15_brainco_hand.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
        
    #fast lio 2 launch nodes
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('g1_nav2_sim_bringup'), 'config', 'g1_bringup.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    #slam params call
    slam_params = os.path.join(
      get_package_share_directory('g1_nav2_sim_bringup'), 'config', 'slam_toolbox_params.yaml'
    )
    
    #nav2 params call
    nav2_params = os.path.join(
      get_package_share_directory('g1_nav2_sim_bringup'), 'config', 'nav2_params.yaml'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='log'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    
    #gazebo pkg and urdf description launch nodes
    pkg_models_path = os.path.join(get_package_share_directory('g1_nav2_sim_bringup'), 'models')
    gazebo_model_path = pkg_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

    gzserver_execute = ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so',
                 os.path.join(get_package_share_directory('g1_nav2_sim_bringup'), 'worlds', 'cafe_no_people.world')],
            additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path},
            output='screen'
    )
    gzclient_execute = ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path},
            output='screen'
    )
    robot_description_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    gazebo_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'g1', '-x', '-3.5', '-y', '2.0', '-z', '0.14', '-R', '0', '-P', '0', '-Y', '0'],
        output='screen'
    )
    
    #pointcloud to laserscan conversion node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/livox/lidar')],
        parameters=[{'target_frame': 'mid360_link'}]
    )
    
    #SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_params]
    )
    
    #nav2 call
    nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('nav2_bringup'), 'launch','navigation_launch.py')
      ),
      launch_arguments={
          'params_file': nav2_params,
          'use_sim_time': 'false'
      }.items()
  )


    

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(TimerAction(period=5.0, actions=[fast_lio_node]))
    ld.add_action(rviz_node)
    
    ld.add_action(robot_description_node)
    ld.add_action(gazebo_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_node)
    ld.add_action(nav2_launch)
    ld.add_action(gzserver_execute)
    ld.add_action(gzclient_execute)

    return ld
    

