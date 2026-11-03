#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
import transforms3d.euler
import math
import numpy as np
import csv
from collections import deque

class PrecisionCircularTracker(Node):
    def __init__(self):
        super().__init__('precision_circular_tracker')
        
        # Controller parameters with optimized defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('radius', 1.0),
                ('target_speed', 0.2),
                ('max_speed', 0.3),
                ('min_speed', 0.05),
                ('max_steering', 1.0),
                ('k_p', 1.5),              # Proportional gain
                ('k_i', 0.05),             # Integral gain
                ('k_d', 0.3),              # Derivative gain
                ('k_theta', 0.8),          # Heading correction gain
                ('k_cte', 0.8),            # Increased cross-track error gain
                ('lookahead_distance', 0.4),
                ('circle_center_x', 1.0),
                ('circle_center_y', 1.0),
                ('lateral_accel_limit', 0.5),
                ('smoothing_factor', 0.2),
                ('path_resolution', 0.05),  # Distance between path points
                ('log_file', '/tmp/smc_cmd_log.csv'),  # Path to CSV log file
            ])
        
        # Initialize CSV logging
        self.log_file = self.get_parameter('log_file').value
        self.csv_writer = None
        self.open_csv_log()

        # State variables
        self.pose = None
        self.velocity = 0.0
        self.yaw_rate = 0.0
        self.integral_cte = 0.0
        self.prev_cte = 0.0
        self.last_time = None
        
        # Path visualization
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'
        self.desired_path = Path()
        self.desired_path.header.frame_id = 'odom'
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.desired_path_pub = self.create_publisher(Path, '/desired_path', 10)
        
        # Command smoothing
        self.smoothed_velocity = 0.0
        self.smoothed_steering = 0.0
        
        # ROS2 setup
        self.cmd_pub = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Initialize desired path
        self.initialize_desired_path()

    def open_csv_log(self):
        """Open the CSV file for writing and initialize the header"""
        try:
            self.csv_file = open(self.log_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['time', 'v', 'omega', 'v_actual', 'omega_actual', 'path_error'])  # Write header
        except Exception as e:
            self.get_logger().error(f"Error opening CSV file: {e}")

    def close_csv_log(self):
        """Close the CSV file gracefully"""
        if self.csv_file:
            self.csv_file.close()

    def initialize_desired_path(self):
        """Create the circular reference path for visualization"""
        radius = self.get_parameter('radius').value
        cx = self.get_parameter('circle_center_x').value
        cy = self.get_parameter('circle_center_y').value
        resolution = self.get_parameter('path_resolution').value
        
        num_points = int(2 * math.pi * radius / resolution)
        self.desired_path.poses = []
        
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = Point(x=x, y=y, z=0.0)
            
            # Calculate tangent orientation
            q = transforms3d.euler.euler2quat(0, 0, angle + math.pi/2, axes='sxyz')
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            self.desired_path.poses.append(pose)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.velocity = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        
        # Publish desired path on first message
        if len(self.desired_path.poses) > 0:
            self.desired_path.header.stamp = self.get_clock().now().to_msg()
            self.desired_path_pub.publish(self.desired_path)

    def get_closest_point(self, cx, cy, radius):
        """Find the closest point on the circle to current position with tangent"""
        dx = self.pose.position.x - cx
        dy = self.pose.position.y - cy
        angle = math.atan2(dy, dx)
        
        return (cx + radius * math.cos(angle),
                cy + radius * math.sin(angle),
                angle + math.pi/2)  # Return tangent angle

    def get_lookahead_point(self, cx, cy, radius, lookahead_dist):
        """Find a point ahead on the circle using arc length"""
        dx = self.pose.position.x - cx
        dy = self.pose.position.y - cy
        angle = math.atan2(dy, dx)
        
        # Calculate lookahead angle based on arc length
        lookahead_angle = lookahead_dist / radius
        
        return (cx + radius * math.cos(angle + lookahead_angle),
                cy + radius * math.sin(angle + lookahead_angle),
                angle + lookahead_angle + math.pi/2)

    def calculate_steering(self, target_x, target_y, target_heading):
        """Improved Stanley controller with better error handling"""
        # Convert target to vehicle coordinates
        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        
        # Transform to vehicle frame
        vehicle_x = dx * math.cos(self.get_yaw()) + dy * math.sin(self.get_yaw())
        vehicle_y = -dx * math.sin(self.get_yaw()) + dy * math.cos(self.get_yaw())
        
        # Cross-track error with sign correction
        cte = vehicle_y
        
        # Heading error with phase unwrapping
        heading_error = math.atan2(math.sin(target_heading - self.get_yaw()),
                                 math.cos(target_heading - self.get_yaw()))
        
        # Adaptive Stanley control law
        k = self.get_parameter('k_cte').value
        speed_adaptive_k = k * (1.0 + 0.5 * abs(self.velocity))  # Increase gain with speed
        
        # Prevent division by zero at low speeds
        denominator = max(0.1, self.velocity)
        steering = heading_error + math.atan2(speed_adaptive_k * cte, denominator)
        
        return steering, cte, heading_error

    def get_yaw(self):
        """Get current yaw angle from quaternion"""
        q = self.pose.orientation
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def limit_lateral_acceleration(self, steering, speed):
        """Limit steering based on lateral acceleration limits"""
        max_steering = self.get_parameter('max_steering').value
        accel_limit = self.get_parameter('lateral_accel_limit').value
        
        if abs(speed) < 0.1:
            return np.clip(steering, -max_steering, max_steering)
        
        max_curvature = accel_limit / (speed**2)
        max_steering = math.atan(max_curvature * 1.0)  # Assuming wheelbase of 1.0
        
        return np.clip(steering, -max_steering, max_steering)

    def control_loop(self):
        if self.pose is None:
            return
            
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Get parameters
        params = {
            'radius': self.get_parameter('radius').value,
            'cx': self.get_parameter('circle_center_x').value,
            'cy': self.get_parameter('circle_center_y').value,
            'lookahead': self.get_parameter('lookahead_distance').value,
            'target_speed': self.get_parameter('target_speed').value,
            'min_speed': self.get_parameter('min_speed').value,
            'max_speed': self.get_parameter('max_speed').value,
        }
        
        # Get target point - using closest point for better accuracy
        target_x, target_y, target_heading = self.get_closest_point(
            params['cx'], params['cy'], params['radius'])
        
        # Calculate steering using improved Stanley method
        steering, cte, heading_error = self.calculate_steering(
            target_x, target_y, target_heading)
        
        # PID control for speed with anti-windup
        speed_error = params['target_speed'] - self.velocity
        self.integral_cte += cte * dt
        self.integral_cte = np.clip(self.integral_cte, -1.0, 1.0)  # Anti-windup
        derivative_cte = (cte - self.prev_cte) / dt if dt > 0 else 0
        self.prev_cte = cte
        
        speed = (self.get_parameter('k_p').value * speed_error +
                self.get_parameter('k_i').value * self.integral_cte +
                self.get_parameter('k_d').value * derivative_cte)
        
        # Apply limits
        speed = np.clip(speed, params['min_speed'], params['max_speed'])
        
        # Limit steering based on lateral acceleration
        steering = self.limit_lateral_acceleration(steering, speed)
        
        # Smooth commands
        smoothing = self.get_parameter('smoothing_factor').value
        self.smoothed_velocity = smoothing * self.smoothed_velocity + (1 - smoothing) * speed
        self.smoothed_steering = smoothing * self.smoothed_steering + (1 - smoothing) * steering
        
        # Publish command
        cmd = TwistStamped()
        cmd.header.stamp = current_time.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = self.smoothed_velocity
        cmd.twist.angular.z = self.smoothed_steering
        self.cmd_pub.publish(cmd)
        
        # Update visualization
        self.update_visualization(target_x, target_y)

        # Log data to CSV
        self.log_data(current_time.to_msg(), self.velocity, self.yaw_rate, speed, steering)

        # Log performance
        self.get_logger().info(
            f"Tracking Error: {cte:.3f}m | Heading Error: {math.degrees(heading_error):.1f}° | "
            f"Speed: {self.velocity:.2f}m/s | Steering: {math.degrees(steering):.1f}°"
        )

    def log_data(self, time_msg, v, omega, v_actual, omega_actual):
        """Log the data to CSV"""
        try:
            # Get actual and desired positions
            xd, yd = self.get_closest_point(self.get_parameter('circle_center_x').value,
                                           self.get_parameter('circle_center_y').value,
                                           self.get_parameter('radius').value)[:2]  # Desired position
            x, y = self.pose.position.x, self.pose.position.y  # Actual position

            # Write time, velocities, and positions to CSV
            self.csv_writer.writerow([time_msg.sec, v, omega, v_actual, omega_actual, x, y, xd, yd, v - v_actual])
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {e}")

    def update_visualization(self, target_x, target_y):
        """Update both actual and target path visualization"""
        # Update actual path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = self.pose
        self.actual_path.poses.append(pose_stamped)
        
        # Keep only the last 500 poses
        if len(self.actual_path.poses) > 500:
            self.actual_path.poses.pop(0)
        
        # Publish paths
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.actual_path)
        
        # Publish desired path periodically
        if len(self.desired_path.poses) > 0:
            self.desired_path.header.stamp = self.get_clock().now().to_msg()
            self.desired_path_pub.publish(self.desired_path)

    def destroy_node(self):
        """Clean up resources before node shutdown"""
        self.close_csv_log()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionCircularTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()