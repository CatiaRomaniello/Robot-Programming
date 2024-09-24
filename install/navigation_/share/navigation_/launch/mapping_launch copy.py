import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    config_dir = '/home/catia/ros2_ws/src/navigation_/config'
    map_file = os.path.join(config_dir,'cappero_laser_odom_diag_2020-05-06-16-26-03.yaml')
    param_file = os.path.join(config_dir,'nav2_param.yaml')
    urdf_file = os.path.join(config_dir, 'robot.urdf')


    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file,
            description='Absolute path to robot URDF file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'use_sim_time': 'true', 'params_file': param_file,
            'map': map_file
            }.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}], 
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-topic', 'robot_description'],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf_file]
        ),
        Node(
            package='navigation_',
            executable='laserScan_',
            name='laserScan_',
            output='screen'
        ),

        Node(
            package='navigation_',
            executable='mobile_base',
            name='mobile_base',
            output='screen'
        ),
        Node(
            package='navigation_',
            executable='ipose',
            name='ipose',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', '/home/catia/ros2_ws/src/navigation_/config/diag1.rviz'],
            output='screen'
        ),
    
    ])