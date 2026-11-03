#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
import transforms3d.euler  # Use transforms3d for quaternion
import math
import numpy as np
import csv

class CircularPIDNode(Node):
    def __init__(self):
        super().__init__('circular_pid_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('radius', 1),
                ('max_lin', 0.1),
                ('min_lin_speed', 0.05),
                ('max_ang', 0.25),
                ('Kp_lin', 1.2),     # LOWERED for smoothness
                ('Ki_lin', 0.05),    # Some I for offset
                ('Kd_lin', 0.3),     # LOWERED for less kick
                ('Kp_ang', 0.6),     # LOWERED for smoothness
                ('Ki_ang', 0.1),    # Small I for heading offset
                ('Kd_ang', 0.05),    # MUCH LOWER for noise
                ('K_cte', 0.6),      # Side error
                ('circle_center_x', 1.0),
                ('circle_center_y', 1.0),
            ])
        self.on_circle = False
        self.join_threshold = 0.10   # meters
        self.t0 = None

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = None

        # PID
        self.prev_lin_error = 0.0
        self.sum_lin_error = 0.0
        self.prev_ang_error = 0.0
        self.sum_ang_error = 0.0
        self.filtered_error_ang = 0.0

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

    def odom_callback(self, msg):
        self.last_odom_msg = msg 
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.t0 = None  # Will be set at join

    def get_desired_state(self, t):
        # User can choose CW (clockwise) or CCW here:
        ccw = True  # Change to False for clockwise

        radius = self.get_parameter('radius').value
        max_lin = self.get_parameter('max_lin').value
        cx = self.get_parameter('circle_center_x').value
        cy = self.get_parameter('circle_center_y').value
        omega = max_lin / radius

        if not self.on_circle:
            # Go to nearest point on circle, face tangent (join phase)
            dx = self.x - cx
            dy = self.y - cy
            angle = math.atan2(dy, dx)
            xd = cx + radius * math.cos(angle)
            yd = cy + radius * math.sin(angle)
            # Pick tangent direction for CCW or CW
            tangent = angle + (math.pi/2 if ccw else -math.pi/2)
            theta_d = tangent
            v_d = 0.0
            omega_d = 0.0
        else:
            # After join: move reference point along the circle at omega
            t_phase = t
            if self.t0 is not None:
                t_phase = t - self.t0
            phi = omega * t_phase if ccw else -omega * t_phase
            xd = cx + radius * math.cos(phi)
            yd = cy + radius * math.sin(phi)
            xd_dot = -radius * omega * math.sin(phi)
            yd_dot =  radius * omega * math.cos(phi)
            theta_d = math.atan2(yd_dot, xd_dot)
            v_d = math.sqrt(xd_dot**2 + yd_dot**2)
            omega_d = omega if ccw else -omega
        return xd, yd, theta_d, v_d, omega_d

    def publish_reference_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        radius = self.get_parameter('radius').value
        cx = self.get_parameter('circle_center_x').value
        cy = self.get_parameter('circle_center_y').value
        max_lin = self.get_parameter('max_lin').value
        omega = max_lin / radius
        num_laps = 3
        num_points = 300
        for i in range(num_points):
            phi = i * (num_laps * 2 * math.pi) / num_points
            xd = cx + radius * math.cos(phi)
            yd = cy + radius * math.sin(phi)
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

    def find_closest_point_on_circle(self, x, y, num_search=200):
        radius = self.get_parameter('radius').value
        cx = self.get_parameter('circle_center_x').value
        cy = self.get_parameter('circle_center_y').value
        min_dist = float('inf')
        best_phi = 0.0
        for i in range(num_search):
            phi = 2 * math.pi * i / num_search
            xd = cx + radius * math.cos(phi)
            yd = cy + radius * math.sin(phi)
            d = math.hypot(x - xd, y - yd)
            if d < min_dist:
                min_dist = d
                best_phi = phi
        return best_phi

    def control_loop(self):
        if self.start_time is None:
            return
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9
        dt = 0.1
        # Parameters
        radius = self.get_parameter('radius').value
        max_lin = self.get_parameter('max_lin').value
        min_lin_speed = self.get_parameter('min_lin_speed').value
        max_ang = self.get_parameter('max_ang').value
        Kp_lin = self.get_parameter('Kp_lin').value
        Ki_lin = self.get_parameter('Ki_lin').value
        Kd_lin = self.get_parameter('Kd_lin').value
        Kp_ang = self.get_parameter('Kp_ang').value
        Ki_ang = self.get_parameter('Ki_ang').value
        Kd_ang = self.get_parameter('Kd_ang').value
        K_cte = self.get_parameter('K_cte').value
        cx = self.get_parameter('circle_center_x').value
        cy = self.get_parameter('circle_center_y').value

        # --- Robust join phase ---
        if not hasattr(self, 'on_path'):
            self.on_path = False
        if not self.on_path:
            # Find closest point on the circle
            phi_closest = self.find_closest_point_on_circle(self.x, self.y)
            xd = cx + radius * math.cos(phi_closest)
            yd = cy + radius * math.sin(phi_closest)
            dx = xd - self.x
            dy = yd - self.y
            error_lin = math.sqrt(dx**2 + dy**2)
            angle_to_path = math.atan2(dy, dx)
            error_ang = math.atan2(math.sin(angle_to_path - self.theta), math.cos(angle_to_path - self.theta))
            v = np.clip(Kp_lin * error_lin, min_lin_speed, max_lin)
            omega = np.clip(Kp_ang * error_ang, -max_ang, max_ang)
            cmd = TwistStamped()
            cmd.header.stamp = current_time.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = omega
            self.pub.publish(cmd)
            self.get_logger().info(f'JOINING: err_lin={error_lin:.3f}, err_ang={math.degrees(error_ang):.1f} deg, v={v:.2f}, omega={omega:.2f}')
            if error_lin < 0.05:  # Threshold for being "on path"
                self.on_path = True
                self.start_time = self.get_clock().now()  # Reset time for reference tracking
                self.get_logger().info('Joined the circle! Now tracking moving reference.')
            self.traj_path.poses.append(self.make_pose_stamped(current_time, self.x, self.y, self.theta))
            self.traj_path.poses = self.traj_path.poses[-1000:]
            self.traj_pub.publish(self.traj_path)
            self.publish_reference_path()
            return

        # --- After join: normal PID tracking ---
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        xd, yd, theta_d, v_d, omega_d = self.get_desired_state(t)
        dx = xd - self.x
        dy = yd - self.y
        error_lin = math.sqrt(dx**2 + dy**2)
        error_ang = math.atan2(math.sin(theta_d - self.theta), math.cos(theta_d - self.theta))
        cte = math.sqrt((self.x - cx)**2 + (self.y - cy)**2) - radius
        if ((self.x - cx) * math.cos(theta_d) + (self.y - cy) * math.sin(theta_d)) < 0:
            cte = -cte

        # === Deadband for angle error ===
        deadband_ang = 0.02  # radians (~1 deg)
        if abs(error_ang) < deadband_ang:
            error_ang = 0.0

        # === Low-pass filter for angular error ===
        self.filtered_error_ang = 0.8 * self.filtered_error_ang + 0.2 * error_ang
        error_ang = self.filtered_error_ang

        d_error_lin = (error_lin - self.prev_lin_error) / dt
        self.sum_lin_error += error_lin * dt
        v = Kp_lin * error_lin + Ki_lin * self.sum_lin_error + Kd_lin * d_error_lin
        self.prev_lin_error = error_lin

        d_error_ang = (error_ang - self.prev_ang_error) / dt
        self.sum_ang_error += error_ang * dt

        # === Clamp integral to prevent windup ===
        max_sum_ang_error = 1.0
        self.sum_ang_error = np.clip(self.sum_ang_error, -max_sum_ang_error, max_sum_ang_error)

        omega = (Kp_ang * error_ang + Ki_ang * self.sum_ang_error + Kd_ang * d_error_ang + K_cte * cte)
        self.prev_ang_error = error_ang

        v = np.clip(abs(v), min_lin_speed, max_lin)
        omega = np.clip(omega, -max_ang, max_ang)

        cmd = TwistStamped()
        cmd.header.stamp = current_time.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.pub.publish(cmd)

        # Log and visualize
        v_actual = 0.0
        omega_actual = 0.0
        if hasattr(self, 'last_odom_msg'):
            v_actual = self.last_odom_msg.twist.twist.linear.x
            omega_actual_raw = self.last_odom_msg.twist.twist.angular.z
            if not hasattr(self, 'filtered_omega_actual'):
                self.filtered_omega_actual = omega_actual_raw
            else:
                self.filtered_omega_actual = 0.9 * self.filtered_omega_actual + 0.1 * omega_actual_raw
            omega_actual = self.filtered_omega_actual
            self.logger.writerow([t, v, omega, v_actual, omega_actual, self.x, self.y, self.theta, xd, yd, theta_d])
        self.log_file.flush()
        self.get_logger().info(
            f"TRACKING: t={t:.1f}s | Target=({xd:.2f},{yd:.2f}) | "
            f"Pos=({self.x:.2f},{self.y:.2f}) | "
            f"err_lin={error_lin:.3f} | err_ang={math.degrees(error_ang):.2f}° | cte={cte:.3f} | v={v:.3f} | omega={omega:.3f}"
        )
        self.traj_path.poses.append(self.make_pose_stamped(current_time, self.x, self.y, self.theta))
        self.traj_path.poses = self.traj_path.poses[-1000:]
        self.traj_pub.publish(self.traj_path)
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
    node = CircularPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.log_file.close()

if __name__ == '__main__':
    main()
