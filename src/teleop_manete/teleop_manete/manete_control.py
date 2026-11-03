import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class DS4CmdVel(Node):
    def __init__(self):
        super().__init__('ds4_cmd_vel')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)

    def joy_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        twist_stamped.header.frame_id = "base_link"  # Optional frame_id

        # Map joystick inputs to velocity
        twist_stamped.twist.linear.x = msg.axes[1] * 1.5  # Left stick vertical (forward/backward)
        twist_stamped.twist.angular.z = msg.axes[3] * 1.0  # Right stick horizontal (rotation)

        self.publisher_.publish(twist_stamped)
        self.get_logger().info(f'Publishing cmd_vel: linear={twist_stamped.twist.linear.x:.2f}, angular={twist_stamped.twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DS4CmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()