import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            TwistStamped,
            '/demo/cmd_vel',
            10
        )

    def callback(self, msg):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        self.pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()