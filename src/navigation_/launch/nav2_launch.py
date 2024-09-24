import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = '/home/catia/ros2_ws/src/navigation_'
    default_model_path = os.path.join(pkg_share, 'config/robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')
    default_map_path = os.path.join(pkg_share, 'config/cappero_laser_odom_diag_2020-05-06-16-26-03.yaml')
    default_nav_params_path = os.path.join(pkg_share, 'config/nav2_param.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    gazebo = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'use_sim_time': 'true', 'params_file': default_nav_params_path,
            'map': default_map_path
            }.items()
        ),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        gazebo
    ])