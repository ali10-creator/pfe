#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math

class CarryPIDNode(Node):
    def __init__(self):
        super().__init__('carry_pid_node')

        # Parameters
        self.goal_points = [(1, 0), (1, 1), (0, 1), (0, 0)]  # square trajectory
        self.current_goal_index = 0

        # PID Gains
        self.Kp_lin = 0.5
        self.Kp_ang = 3.0  # increased for faster rotation
        self.Kd_ang = 0.4

        # Limits
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.5

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_angle_error = 0.0

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Carry PID node started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def control_loop(self):
        if self.current_goal_index >= len(self.goal_points):
            return

        goal_x, goal_y = self.goal_points[self.current_goal_index]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_theta - self.theta), math.cos(target_theta - self.theta))

        # PID for angular velocity
        derivative = (angle_error - self.prev_angle_error) / 0.1
        omega = self.Kp_ang * angle_error + self.Kd_ang * derivative
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))
        self.prev_angle_error = angle_error

        # Smooth forward motion even if angle error is large
        alignment_factor = max(0.0, 1.0 - abs(angle_error) / math.pi)
        v = self.Kp_lin * distance * alignment_factor
        v = min(self.max_linear_speed, v)

        # Publish command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Goal {self.current_goal_index+1}: pos=({self.x:.2f},{self.y:.2f}) → ({goal_x:.2f},{goal_y:.2f}) | d={distance:.2f} θ_err={math.degrees(angle_error):.1f}° v={v:.2f} ω={omega:.2f}"
        )

        # Move to next goal
        if distance < 0.1:
            self.current_goal_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CarryPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Carry PID node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
