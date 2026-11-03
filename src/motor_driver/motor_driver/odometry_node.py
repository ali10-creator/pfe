#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import re

def normalize_angle_rad(angle):
    """Normalize any angle in radians to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class OdometryIMUNode(Node):
    def __init__(self):
        super().__init__('odometry_imu_node')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1651 / 2)      # meters
        self.declare_parameter('wheel_base', 0.3)               # meters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('dt', 0.1)                       # seconds

        # Calibration parameters
        self.declare_parameter('forward_scale', 1.0)
        self.declare_parameter('backward_scale', 1.2)
        self.forward_scale = self.get_parameter('forward_scale').value
        self.backward_scale = self.get_parameter('backward_scale').value

        self.radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.dt = self.get_parameter('dt').value

        self.x = 0.0
        self.y = 0.0
        self.yaw_rad = 0.0  # Current yaw in radians

        try:
            self.ser = serial.Serial(self.get_parameter('serial_port').value, 115200, timeout=1)
            self.get_logger().info("✅ Serial connected to Arduino.")
        except Exception as e:
            self.get_logger().error(f"❌ Could not open serial port: {e}")
            raise SystemExit

        self.odom_pub = self.create_publisher(Odometry, '/demo/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.update_odometry)

    def update_odometry(self):
        try:
            line = self.ser.readline().decode().strip()
            # Expecting: L:<rpmL> R:<rpmR> YAW:<yaw>
            match = re.match(r"L:(-?\d+\.?\d*) R:(-?\d+\.?\d*) YAW:(-?\d+\.?\d*)", line)
            if not match:
                return

            rpmL = float(match.group(1))
            rpmR = float(match.group(2))
            yaw_deg = float(match.group(3))

            # Convert yaw to radians and normalize to [-pi, pi]
            yaw_rad = math.radians(yaw_deg)
            self.yaw_rad = normalize_angle_rad(yaw_rad)

            # Calculate wheel velocities (m/s)
            vL = (2 * math.pi * self.radius * rpmL) / 60.0
            vR = (2 * math.pi * self.radius * rpmR) / 60.0
            v = (vL + vR) / 2.0
            omega = (vR - vL) / self.wheel_base

            # Calibration: adjust forward/backward travel if needed
            if v > 0:
                v *= self.forward_scale
            elif v < 0:
                v *= self.backward_scale

            # Integrate position using IMU yaw
            self.x += v * math.cos(self.yaw_rad) * self.dt
            self.y += v * math.sin(self.yaw_rad) * self.dt

            # Quaternion from yaw
            qz = math.sin(self.yaw_rad / 2.0)
            qw = math.cos(self.yaw_rad / 2.0)
            quat = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

            # Publish odometry
            odom = Odometry()
            now = self.get_clock().now().to_msg()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quat
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = omega

            self.odom_pub.publish(odom)

            # Broadcast TF odom -> base_link
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().info(
                f"📦 x={self.x:.3f} y={self.y:.3f} | YAW(IMU)={math.degrees(self.yaw_rad):.1f}° | v={v:.2f} m/s | ω={omega:.2f} rad/s"
            )

        except Exception as e:
            self.get_logger().warn(f"⚠️ Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
