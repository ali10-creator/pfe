import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
import numpy as np
import math

class ControllerSignalPlotter(Node):
    def __init__(self):
        super().__init__('controller_signal_plotter')

        # Initialize command velocities
        self.v_cmd = 0.0  # Default command velocity
        self.omega_cmd = 0.0  # Default angular velocity
        self.radius = 1.0  # Radius of the circular path
        self.cx = 1.0  # X coordinate of the circle's center
        self.cy = 1.0  # Y coordinate of the circle's center

        # Create subscribers to listen to command velocities and odometry
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/demo/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odom_callback,
            10
        )

        # Initialize lists to store the values for plotting
        self.times = []
        self.v_cmds = []
        self.v_actuals = []
        self.omega_cmds = []
        self.omega_actuals = []
        self.velocity_errors = []
        self.position_errors = []

        # Initialize the plot
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 12))
        self.fig.suptitle("Live SMC Command vs Actual Velocity, Errors, and Position Error")

        # Titles for individual subplots
        self.axs[0].set_title("Linear Velocity (v) vs Time")
        self.axs[1].set_title("Angular Velocity (ω) vs Time")
        self.axs[2].set_title("Velocity Error (v_cmd - v_actual) vs Time")
        self.axs[3].set_title("Position Error (Euclidean) vs Time")

        # Labels for each subplot
        self.axs[0].set_ylabel("Linear Velocity (v) [m/s]")
        self.axs[1].set_ylabel("Angular Velocity (ω) [rad/s]")
        self.axs[2].set_ylabel("Velocity Error (v_cmd - v_actual)")
        self.axs[3].set_ylabel("Position Error (Euclidean) [m]")

        # Adding grid to all subplots
        for ax in self.axs:
            ax.grid(True)  # Enable grid

        # Add space between the subplots
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout to make space for the main title

        plt.ion()  # Interactive mode on
        plt.show()

    def cmd_vel_callback(self, msg):
        """Callback to update command velocities."""
        self.v_cmd = msg.twist.linear.x
        self.omega_cmd = msg.twist.angular.z

    def odom_callback(self, msg):
        """Callback to update odometry and calculate errors."""
        v_actual = msg.twist.twist.linear.x
        omega_actual = msg.twist.twist.angular.z
        
        # Calculate position error (Euclidean distance to the closest point on the path)
        closest_x, closest_y = self.get_closest_point(self.cx, self.cy, self.radius, msg.pose.pose.position.x, msg.pose.pose.position.y)
        position_error = np.sqrt((msg.pose.pose.position.x - closest_x)**2 + (msg.pose.pose.position.y - closest_y)**2)

        # Store data for plotting
        self.v_actuals.append(v_actual)
        self.omega_actuals.append(omega_actual)
        self.velocity_errors.append(self.v_cmd - v_actual)  # Velocity error
        self.position_errors.append(position_error)  # Position error

        # Add command velocities
        self.v_cmds.append(self.v_cmd)
        self.omega_cmds.append(self.omega_cmd)

        # Time tracking
        self.times.append(self.get_clock().now().nanoseconds / 1e9)

        # Update the plot
        self.update_plot()

    def get_closest_point(self, cx, cy, radius, x, y):
        """Calculate the closest point on the circular path to the robot's current position."""
        dx = x - cx
        dy = y - cy
        angle = math.atan2(dy, dx)
        closest_x = cx + radius * math.cos(angle)
        closest_y = cy + radius * math.sin(angle)
        return closest_x, closest_y

    def update_plot(self):
        """Update the plot with new data."""
        # Clear previous plots
        self.axs[0].cla()
        self.axs[1].cla()
        self.axs[2].cla()
        self.axs[3].cla()

        # Plot the data
        # Divide the actual linear velocity by 3 only for plotting (as per your request)
        self.axs[0].plot(self.times, self.v_cmds, label="Command v [m/s]", linestyle='-', color='tab:blue')
        self.axs[0].plot(self.times, [v / 3 for v in self.v_actuals], label="Actual v [m/s] (divided by 3)", linestyle='--', color='tab:orange')
        self.axs[1].plot(self.times, self.omega_cmds, label="Command ω [rad/s]", linestyle='-', color='tab:green')
        self.axs[1].plot(self.times, self.omega_actuals, label="Actual ω [rad/s]", linestyle='--', color='tab:red')
        self.axs[2].plot(self.times, self.velocity_errors, label="Linear Error (v - v_actual)", linestyle='-', color='tab:brown')
        self.axs[2].plot(self.times, [self.v_cmd - omega for omega in self.omega_actuals], label="Angular Error (ω - ω_actual)", linestyle='-', color='tab:orange')
        self.axs[3].plot(self.times, self.position_errors, label="Position Error (Euclidean)", linestyle='-', color='tab:purple')

        # Set legends
        self.axs[0].legend()
        self.axs[1].legend()
        self.axs[2].legend()
        self.axs[3].legend()

        plt.draw()
        plt.pause(0.1)  # Pause to update the plot

    def destroy_node(self):
        """Clean up resources before node shutdown."""
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSignalPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
