#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # === Launch Arguments ===
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # === Paths ===
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf', 'ali.urdf')

    rviz_config_path = os.path.join(
        get_package_share_directory('robot_description'),
        'rviz', 'robot_with_lidar.rviz')

    return LaunchDescription([
        # === Declare Launch Arguments ===
        DeclareLaunchArgument('channel_type', default_value=channel_type),
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('inverted', default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),

        # === Robot State Publisher ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(robot_description_path).read(),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # === Joint State Publisher ===
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # === SLLIDAR Node ===
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        # === RViz2 ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # === Joystick Node (fixed configuration) ===
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 1
            }],
            output='screen'
        ),

        # === Teleop Node ===
        Node(
            package='teleop_manete',
            executable='joy_cmd_vel',
            name='joystick_teleop',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),


    ])
