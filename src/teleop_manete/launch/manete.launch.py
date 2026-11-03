from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy = Node(
        package="joy",
        executable="joy_node"
    )
    controler = Node(
        package="teleop_manete",
        executable='lyes'
    )
    ld.add_action(joy)
    ld.add_action(controler)
    return ld