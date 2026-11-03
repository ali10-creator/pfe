from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess



def generate_launch_description():

    pkg_share = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ali.sdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    gazebo_config_path = os.path.join(get_package_share_path('robot_bringup'),'config', 'gazebo_bridge.yaml')

    # Launch Gazebo (Harmonic)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': open(urdf_path).read()}, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description','-Y','-4.71'],
        output='screen'
    )


    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    )




    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(get_package_share_path('robot_bringup'),'config', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
        # SMC infinity node
    pid_infiny = Node(
        package='motor_driver',  # Change this to your actual ROS2 Python package name
        executable='pid_infiny',  # Entry point name from setup.py
        name='pid_infiny',
        output='screen',
        parameters=[os.path.join(get_package_share_path('motor_driver'), 'config', 'smc_circular_params.yaml')]
    )

    plot = Node(
        package='motor_driver',  # Change this to your actual ROS2 Python package name
        executable='pid_plot_infiny',  # Entry point name from setup.py
        name='pid_plot_infiny',
        output='screen'
    )

   



    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        bridge_node,
        robot_localization_node,
        rviz2_node,
        pid_infiny,
        plot
        
     
    ])

