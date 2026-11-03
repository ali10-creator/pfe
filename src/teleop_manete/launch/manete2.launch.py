from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Joystick input node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',  # Default joystick device
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Joystick-to-cmd_vel mapper
    joy_cmd_vel_node = Node(
        package='teleop_manete',
        executable='joy_cmd_vel',  # Must match setup.py entry point
        name='joy_cmd_vel',
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(joy_node)
    ld.add_action(joy_cmd_vel_node)

    return ld
