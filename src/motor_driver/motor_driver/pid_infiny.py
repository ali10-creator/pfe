#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import transforms3d.euler  # Use transforms3d for quaternion
import math
import numpy as np
import csv

class InfinityPIDNode(Node):
    def __init__(self):
        super().__init__('infinity_pid_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scale_x', 1.5),           # Scale factor for X-axis of infinity
                ('scale_y', 0.7),           # Scale factor for Y-axis of infinity  
                ('max_lin', 0.12),          # Maximum linear velocity
                ('min_lin_speed', 0.03),    # Minimum linear velocity
                ('max_ang', 0.6),           # Maximum angular velocity
                ('Kp_lin', 1.2),            # Linear proportional gain
                ('Ki_lin', 0.0),            # Linear integral gain
                ('Kd_lin', 0.1),            # Linear derivative gain
                ('Kp_ang', 1.5),            # Angular proportional gain
                ('Ki_ang', 0.0),            # Angular integral gain  
                ('Kd_ang', 0.2),            # Angular derivative gain
                ('center_x', 1.0),          # Center X of infinity trajectory
                ('center_y', 1.0),          # Center Y of infinity trajectory
                ('period', 40.0),           # Time period for one complete infinity loop
                ('speed_factor', 0.3),      # Speed factor for trajectory progression
            ])
        
        # States
        self.on_trajectory = False
        self.join_threshold = 0.15   # Distance threshold to consider "on trajectory"
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = None
        self.trajectory_start_time = None

        # PID variables
        self.prev_lin_error = 0.0
        self.sum_lin_error = 0.0
        self.prev_ang_error = 0.0
        self.sum_ang_error = 0.0

        # Trajectory time parameter
        self.trajectory_time = 0.0

        # Path for visualization
        self.traj_path = Path()
        self.traj_path.header.frame_id = 'odom'
        self.traj_pub = self.create_publisher(Path, '/robot_path', 1)

        # Logging
        self.log_file = open('/tmp/pid_cmd_log.csv', 'w', newline='')
        self.logger = csv.writer(self.log_file)
        self.logger.writerow(['time', 'v', 'omega', 'v_actual', 'omega_actual', 'x', 'y', 'theta', 'xd', 'yd', 'theta_d'])

        # ROS2
        self.pub = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Infinity PID Controller Started - Robot should start at (0,0)")

    def odom_callback(self, msg):
        self.last_odom_msg = msg 
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Robot starting position: ({self.x:.3f}, {self.y:.3f})")

    def get_infinity_point(self, t):
        """
        Generate infinity (figure-8) trajectory point
        Parametric equations for figure-8:
        x(t) = scale_x * sin(omega*t)
        y(t) = scale_y * sin(2*omega*t)
        """
        scale_x = self.get_parameter('scale_x').value
        scale_y = self.get_parameter('scale_y').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        period = self.get_parameter('period').value
        
        omega = 2 * math.pi / period  # Angular frequency
        
        # Position on infinity trajectory
        xd = center_x + scale_x * math.sin(omega * t)
        yd = center_y + scale_y * math.sin(2 * omega * t)
        
        # Velocity (first derivatives) for orientation
        xd_dot = scale_x * omega * math.cos(omega * t)
        yd_dot = scale_y * 2 * omega * math.cos(2 * omega * t)
        
        # Desired orientation (tangent to trajectory)
        theta_d = math.atan2(yd_dot, xd_dot)
        
        return xd, yd, theta_d

    def find_closest_point_on_trajectory(self, robot_x, robot_y):
        """Find the parameter t for the closest point on trajectory to robot position"""
        period = self.get_parameter('period').value
        
        min_dist = float('inf')
        best_t = 0.0
        
        # Search over one complete period with fine resolution
        num_samples = 300
        for i in range(num_samples):
            t = period * i / num_samples
            xd, yd, _ = self.get_infinity_point(t)
            dist = math.sqrt((robot_x - xd)**2 + (robot_y - yd)**2)
            if dist < min_dist:
                min_dist = dist
                best_t = t
                
        return best_t, min_dist

    def publish_reference_path(self):
        """Publish the complete infinity trajectory for visualization"""
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
        
        # Get parameters
        max_lin = self.get_parameter('max_lin').value
        min_lin_speed = self.get_parameter('min_lin_speed').value
        max_ang = self.get_parameter('max_ang').value
        Kp_lin = self.get_parameter('Kp_lin').value
        Ki_lin = self.get_parameter('Ki_lin').value
        Kd_lin = self.get_parameter('Kd_lin').value
        Kp_ang = self.get_parameter('Kp_ang').value
        Ki_ang = self.get_parameter('Ki_ang').value
        Kd_ang = self.get_parameter('Kd_ang').value
        speed_factor = self.get_parameter('speed_factor').value
        
        # PHASE 1: JOIN THE TRAJECTORY
        if not self.on_trajectory:
            # Find closest point on trajectory
            closest_t, min_distance = self.find_closest_point_on_trajectory(self.x, self.y)
            target_x, target_y, target_theta = self.get_infinity_point(closest_t)
            
            # Calculate errors
            dx = target_x - self.x
            dy = target_y - self.y
            distance_error = math.sqrt(dx**2 + dy**2)
            
            # Angle to target point
            angle_to_target = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(angle_to_target - self.theta), 
                                   math.cos(angle_to_target - self.theta))
            
            # Simple proportional control to reach the trajectory
            v = Kp_lin * distance_error
            omega = Kp_ang * angle_error
            
            # Apply limits
            v = np.clip(v, min_lin_speed, max_lin * 0.8)
            omega = np.clip(omega, -max_ang * 0.8, max_ang * 0.8)
            
            # Publish command
            cmd = TwistStamped()
            cmd.header.stamp = current_time.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = omega
            self.pub.publish(cmd)
            
            self.get_logger().info(f'JOINING: distance_to_trajectory={distance_error:.3f}m, angle_error={math.degrees(angle_error):.1f}°, v={v:.3f}, ω={omega:.3f}')
            
            # Check if we've joined the trajectory
            if distance_error < self.join_threshold:
                self.on_trajectory = True
                self.trajectory_start_time = self.get_clock().now()
                self.trajectory_time = closest_t  # Start from closest point
                # Reset PID accumulators
                self.sum_lin_error = 0.0
                self.sum_ang_error = 0.0
                self.prev_lin_error = 0.0
                self.prev_ang_error = 0.0
                self.get_logger().info(f'JOINED TRAJECTORY! Starting to follow from t={closest_t:.2f}')
            
        # PHASE 2: FOLLOW THE TRAJECTORY
        else:
            # Update trajectory time (moving reference point)
            elapsed_time = (current_time - self.trajectory_start_time).nanoseconds / 1e9
            self.trajectory_time += speed_factor * dt  # Move reference point along trajectory
            
            # Get current target point on trajectory
            target_x, target_y, target_theta = self.get_infinity_point(self.trajectory_time)
            
            # Calculate position errors
            dx = target_x - self.x
            dy = target_y - self.y
            position_error = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle error
            angle_error = math.atan2(math.sin(target_theta - self.theta), 
                                   math.cos(target_theta - self.theta))
            
            # PID for linear velocity (minimize position error)
            d_position_error = (position_error - self.prev_lin_error) / dt
            self.sum_lin_error += position_error * dt
            # Limit integral windup
            self.sum_lin_error = np.clip(self.sum_lin_error, -2.0, 2.0)
            
            v = (Kp_lin * position_error + 
                 Ki_lin * self.sum_lin_error + 
                 Kd_lin * d_position_error)
            self.prev_lin_error = position_error
            
            # PID for angular velocity (minimize orientation error)
            d_angle_error = (angle_error - self.prev_ang_error) / dt
            self.sum_ang_error += angle_error * dt
            # Limit integral windup
            self.sum_ang_error = np.clip(self.sum_ang_error, -1.0, 1.0)
            
            omega = (Kp_ang * angle_error + 
                    Ki_ang * self.sum_ang_error + 
                    Kd_ang * d_angle_error)
            self.prev_ang_error = angle_error
            
            # Apply velocity limits
            v = np.clip(abs(v), min_lin_speed, max_lin)
            omega = np.clip(omega, -max_ang, max_ang)
            
            # Publish command
            cmd = TwistStamped()
            cmd.header.stamp = current_time.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = omega
            self.pub.publish(cmd)
            
            # Log data
            v_actual = 0.0
            omega_actual = 0.0
            if hasattr(self, 'last_odom_msg'):
                v_actual = self.last_odom_msg.twist.twist.linear.x
                omega_actual = self.last_odom_msg.twist.twist.angular.z
                self.logger.writerow([elapsed_time, v, omega, v_actual, omega_actual, 
                                     self.x, self.y, self.theta, target_x, target_y, target_theta])
            self.log_file.flush()
            
            self.get_logger().info(
                f"TRACKING: t={self.trajectory_time:.1f} | Target=({target_x:.2f},{target_y:.2f}) | "
                f"Robot=({self.x:.2f},{self.y:.2f}) | pos_err={position_error:.3f}m | "
                f"ang_err={math.degrees(angle_error):.1f}° | v={v:.3f} | ω={omega:.3f}"
            )
        
        # Update visualization
        self.traj_path.poses.append(self.make_pose_stamped(current_time, self.x, self.y, self.theta))
        self.traj_path.poses = self.traj_path.poses[-1000:]  # Keep last 1000 points
        self.traj_pub.publish(self.traj_path)
        
        # Publish reference path occasionally  
        total_time = (current_time - self.start_time).nanoseconds / 1e9
        if int(total_time * 10) % 30 == 0:  # Every 3 seconds
            self.publish_reference_path()

    def make_pose_stamped(self, stamp, x, y, theta):
        """Helper function to create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = transforms3d.euler.euler2quat(0, 0, theta, axes='sxyz')
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        if hasattr(self, 'log_file'):
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        infinity_pid = InfinityPIDNode()
        rclpy.spin(infinity_pid)
    except KeyboardInterrupt:
        infinity_pid.get_logger().info("Stopped by user")
    finally:
        if 'infinity_pid' in locals():
            infinity_pid.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()