#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class DS4CmdVel(Node):
    def __init__(self):
        super().__init__('ds4_cmd_vel')

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)

    def joy_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"

        # Map joystick inputs to velocity
        twist_stamped.twist.linear.x = msg.axes[1] * 0.4
        twist_stamped.twist.angular.z = msg.axes[3] * 1.0

        self.publisher_.publish(twist_stamped)

        self.get_logger().info(
            f'🎮 /demo/cmd_vel: linear.x={twist_stamped.twist.linear.x:.2f}, angular.z={twist_stamped.twist.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = DS4CmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
