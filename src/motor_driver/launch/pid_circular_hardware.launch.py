#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'lyes.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    
    return LaunchDescription([
        
        # Use real time (not simulation time)
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false', 
            description='Use simulation time when false (real robot)'
        ),
        
        # Robot State Publisher - for visualization
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_viz",
            parameters=[{
                'robot_description': open(urdf_path).read(),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ),
        
        # RViz2 - Main visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ),

        # PID Circular Control Node
        Node(
            package='motor_driver',
            executable='pid_circular_node',
            name='pid_circular_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # PID Plot Node for visualization
        Node(
            package='motor_driver',
            executable='pid_plot',
            name='pid_plot',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])