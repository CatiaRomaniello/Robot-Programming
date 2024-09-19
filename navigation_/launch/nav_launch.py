from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo pub_occupancy_grid
        Node(
            package='navigation_',
            executable='pub_occupancy_grid',
            name='occupancy_grid',
            output='screen'
        ),
        
        # Nodo laser_scan
        Node(
            package='navigation_',
            executable='laserScan_',
            name='laser_scan',
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
            executable='tf_node',
            name='tf_node',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'src/navigation_/config/default.rviz']  
        ),
    ])
