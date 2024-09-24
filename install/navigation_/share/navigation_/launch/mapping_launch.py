import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = '/home/catia/ros2_ws/src/navigation_/config'
    map_file = os.path.join(config_dir,'cappero_laser_odom_diag_2020-05-06-16-26-03.yaml')
    param_file = os.path.join(config_dir,'nav2_param.yaml')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'use_sim_time': 'false', 'params_file': param_file,
            'map': map_file
            }.items()
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
            arguments=['-d', '/home/catia/ros2_ws/src/navigation_/config/diag.rviz'],
            output='screen'
        ),
    
    ])