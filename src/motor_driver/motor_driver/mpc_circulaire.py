#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np
from scipy.optimize import minimize
import math

class CircularMPCNode(Node):
    def __init__(self):
        super().__init__('circular_mpc_node')

        # Circle parameters
        self.radius = 1.0  # meters
        self.omega = 0.1   # rad/s (réduit)
        self.center_x = 0.0
        self.center_y = 0.0
        self.duration = (2 * math.pi) / self.omega

        # MPC parameters
        self.horizon = 8
        self.dt = 0.1
        self.control_rate = 10  # Hz

        # Constraints
        self.max_vel = 0.2      # Réduit
        self.max_ang_vel = 0.3  # Réduit
        self.min_vel = 0.03

        # Weights
        self.Q = np.diag([5.0, 5.0, 2.0])
        self.R = np.diag([0.5, 0.5])

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = self.get_clock().now()
        self.stopped = False

        # ROS
        self.pub = self.create_publisher(TwistStamped, '/demo/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(1.0/self.control_rate, self.mpc_control_loop)

        self.get_logger().info("Circular MPC Controller Initialized")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.theta = math.atan2(siny, cosy)

    def get_reference_trajectory(self, t, steps):
        ref_states = []
        for i in range(steps):
            time = t + i*self.dt
            x_ref = self.center_x + self.radius * math.cos(self.omega * time)
            y_ref = self.center_y + self.radius * math.sin(self.omega * time)
            theta_ref = math.atan2(
                self.omega * math.cos(self.omega * time),
                -self.omega * math.sin(self.omega * time)
            )
            ref_states.append([x_ref, y_ref, theta_ref])
        return np.array(ref_states)

    def vehicle_model(self, state, control):
        theta = state[2]
        v = control[0]
        w = control[1]
        dx = v * math.cos(theta)
        dy = v * math.sin(theta)
        dtheta = w
        return np.array([dx, dy, dtheta])

    def predict_trajectory(self, init_state, controls):
        states = [init_state]
        current_state = init_state.copy()
        for control in controls:
            derivative = self.vehicle_model(current_state, control)
            next_state = current_state + derivative * self.dt
            states.append(next_state)
            current_state = next_state
        return np.array(states)

    def cost_function(self, controls_flat, *args):
        init_state, ref_traj = args
        controls = controls_flat.reshape(-1, 2)
        pred_traj = self.predict_trajectory(init_state, controls)
        cost = 0.0
        for i in range(len(ref_traj)):
            pos_error = pred_traj[i][:2] - ref_traj[i][:2]
            cost += pos_error.T @ self.Q[:2,:2] @ pos_error
            heading_error = pred_traj[i][2] - ref_traj[i][2]
            cost += self.Q[2,2] * heading_error**2
            cost += controls[i].T @ self.R @ controls[i]
        if len(controls) > 1:
            for i in range(len(controls)-1):
                cost += 0.1 * np.sum((controls[i+1] - controls[i])**2)
        return cost

    def mpc_control_loop(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        if t > self.duration:
            if not self.stopped:
                stop = TwistStamped()
                stop.header.stamp = now.to_msg()
                stop.header.frame_id = 'base_link'
                self.pub.publish(stop)
                self.get_logger().info("✅ Full circle complete. Robot stopped.")
                self.stopped = True
            return

        current_state = np.array([self.x, self.y, self.theta])
        ref_traj = self.get_reference_trajectory(t, self.horizon)

        init_controls = np.zeros((self.horizon, 2))
        init_controls[:, 0] = self.max_vel * 0.5  # Réduit à 50%
        init_controls[:, 1] = self.omega

        bounds = [
            (self.min_vel, self.max_vel) if i%2 == 0 else (-self.max_ang_vel, self.max_ang_vel)
            for i in range(2*self.horizon)
        ]

        result = minimize(
            self.cost_function,
            init_controls.flatten(),
            args=(current_state, ref_traj),
            bounds=bounds,
            method='SLSQP',
            options={'maxiter': 100, 'ftol': 1e-6}
        )

        if result.success:
            optimal_controls = result.x.reshape(-1, 2)
            control_cmd = optimal_controls[0]
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = float(control_cmd[0])
            cmd.twist.angular.z = float(control_cmd[1])
            self.pub.publish(cmd)

            self.get_logger().info(
                f"MPC: v={control_cmd[0]:.2f}, ω={control_cmd[1]:.2f} | "
                f"Pos: ({self.x:.2f}, {self.y:.2f}) | "
                f"Time: {t:.1f}/{self.duration:.1f}s | Cost: {result.fun:.2f}"
            )
        else:
            self.get_logger().warn(f"MPC failed! Using fallback ({result.message})")
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.header.frame_id = 'base_link'

            dx = self.center_x + self.radius * math.cos(self.omega * t) - self.x
            dy = self.center_y + self.radius * math.sin(self.omega * t) - self.y
            distance_error = math.hypot(dx, dy)
            heading_error = math.atan2(dy, dx) - self.theta

            cmd.twist.linear.x = float(min(self.max_vel * 0.4, 0.2 * distance_error))
            cmd.twist.angular.z = float(np.clip(0.3 * heading_error, -0.2, 0.2))
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CircularMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
