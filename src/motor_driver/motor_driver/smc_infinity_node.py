#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
import transforms3d.euler
import math
import numpy as np
import csv

class InfinitySMCNode(Node):
    def __init__(self):
        super().__init__('infinity_smc_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('scale_x', 1.0),
                ('scale_y', 0.5),
                ('max_lin', 0.12),
                ('min_lin_speed', 0.03),
                ('max_ang', 0.6),
                ('center_x', 1.0),
                ('center_y', 1.0),
                ('period', 40.0),
                ('speed_factor', 0.3),
                # SMC gains
                ('lambda_theta', 2.0),
                ('k_theta', 4.0),
                ('k_p', 6.0),
            ])
        
        self.on_trajectory = False
        self.join_threshold = 0.15
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = None
        self.trajectory_start_time = None
        self.trajectory_time = 0.0

        # For visualization
        self.traj_path = Path()
        self.traj_path.header.frame_id = 'odom'
        self.traj_pub = self.create_publisher(Path, '/robot_path', 1)

        # Logging: now with position_error
        self.log_file = open('/tmp/smc_cmd_log.csv', 'w', newline='')
        self.logger = csv.writer(self.log_file)
        self.logger.writerow([
            'time', 'v', 'omega', 'v_actual', 'omega_actual',
            'x', 'y', 'theta', 'xd', 'yd', 'theta_d', 'position_error'
        ])

        # ROS 2
        self.pub = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Infinity SMC Controller Started")

    def odom_callback(self, msg):
        self.last_odom_msg = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Robot starting position: ({self.x:.3f}, {self.y:.3f})")

    def get_infinity_point(self, t):
        scale_x = self.get_parameter('scale_x').value
        scale_y = self.get_parameter('scale_y').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        period = self.get_parameter('period').value
        omega = 2 * math.pi / period

        xd = center_x + scale_x * math.sin(omega * t)
        yd = center_y + scale_y * math.sin(2 * omega * t)
        xd_dot = scale_x * omega * math.cos(omega * t)
        yd_dot = scale_y * 2 * omega * math.cos(2 * omega * t)
        theta_d = math.atan2(yd_dot, xd_dot)
        return xd, yd, theta_d

    def find_closest_point_on_trajectory(self, robot_x, robot_y):
        period = self.get_parameter('period').value
        min_dist = float('inf')
        best_t = 0.0
        num_samples = 300
        for i in range(num_samples):
            t = period * i / num_samples
            xd, yd, _ = self.get_infinity_point(t)
            dist = math.sqrt((robot_x - xd) ** 2 + (robot_y - yd) ** 2)
            if dist < min_dist:
                min_dist = dist
                best_t = t
        return best_t, min_dist

    def publish_reference_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        period = self.get_parameter('period').value
        num_points = 400
        for i in range(num_points):
            t = period * i / num_points
            xd, yd, _ = self.get_infinity_point(t)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = xd
            pose.pose.position.y = yd
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        if not hasattr(self, 'ref_path_pub'):
            self.ref_path_pub = self.create_publisher(Path, '/desired_path', 1)
        self.ref_path_pub.publish(path_msg)

    def control_loop(self):
        if self.start_time is None:
            return

        current_time = self.get_clock().now()
        dt = 0.1
        max_lin = self.get_parameter('max_lin').value
        min_lin_speed = self.get_parameter('min_lin_speed').value
        max_ang = self.get_parameter('max_ang').value
        speed_factor = self.get_parameter('speed_factor').value
        lambda_theta = self.get_parameter('lambda_theta').value
        k_theta = self.get_parameter('k_theta').value
        k_p = self.get_parameter('k_p').value

        # === PHASE 1: JOIN THE TRAJECTORY ===
        if not self.on_trajectory:
            closest_t, min_distance = self.find_closest_point_on_trajectory(self.x, self.y)
            target_x, target_y, target_theta = self.get_infinity_point(closest_t)
            dx = target_x - self.x
            dy = target_y - self.y
            distance_error = math.sqrt(dx ** 2 + dy ** 2)
            angle_to_target = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(angle_to_target - self.theta), math.cos(angle_to_target - self.theta))
            v = 0.8 * distance_error
            omega = 1.5 * angle_error
            v = np.clip(v, min_lin_speed, max_lin * 0.8)
            omega = np.clip(omega, -max_ang * 0.8, max_ang * 0.8)
            cmd = TwistStamped()
            cmd.header.stamp = current_time.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = omega
            self.pub.publish(cmd)
            if distance_error < self.join_threshold:
                self.on_trajectory = True
                self.trajectory_start_time = self.get_clock().now()
                self.trajectory_time = closest_t
        else:
            elapsed_time = (current_time - self.trajectory_start_time).nanoseconds / 1e9
            self.trajectory_time += speed_factor * dt
            target_x, target_y, target_theta = self.get_infinity_point(self.trajectory_time)
            dx = target_x - self.x
            dy = target_y - self.y
            position_error = math.sqrt(dx ** 2 + dy ** 2)
            # Error in robot frame
            err_x = math.cos(self.theta) * dx + math.sin(self.theta) * dy
            err_y = -math.sin(self.theta) * dx + math.cos(self.theta) * dy

            # SMC law for tracking:
            v = max_lin * np.tanh(k_p * err_x)
            if abs(err_x) < 0.05:
                v = min_lin_speed

            angle_to_target = math.atan2(dy, dx)
            heading_error = math.atan2(math.sin(angle_to_target - self.theta), math.cos(angle_to_target - self.theta))
            s_theta = heading_error + lambda_theta * err_y
            omega = max_ang * np.tanh(k_theta * s_theta)

            v = np.clip(v, min_lin_speed, max_lin)
            omega = np.clip(omega, -max_ang, max_ang)

            cmd = TwistStamped()
            cmd.header.stamp = current_time.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = omega
            self.pub.publish(cmd)

            # Logging (now with position_error)
            v_actual = 0.0
            omega_actual = 0.0
            if hasattr(self, 'last_odom_msg'):
                v_actual = self.last_odom_msg.twist.twist.linear.x
                omega_actual = self.last_odom_msg.twist.twist.angular.z
                self.logger.writerow([
                    elapsed_time, v, omega, v_actual, omega_actual,
                    self.x, self.y, self.theta, target_x, target_y, target_theta, position_error
                ])
            self.log_file.flush()

        # Visualization
        self.traj_path.poses.append(self.make_pose_stamped(current_time, self.x, self.y, self.theta))
        self.traj_path.poses = self.traj_path.poses[-1000:]
        self.traj_pub.publish(self.traj_path)
        total_time = (current_time - self.start_time).nanoseconds / 1e9
        if int(total_time * 10) % 30 == 0:
            self.publish_reference_path()

    def make_pose_stamped(self, stamp, x, y, theta):
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, theta, axes='sxyz')
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def destroy_node(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = InfinitySMCNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
