#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

import sys
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Try to import GPIO
        self.gpio_available = True
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self.get_logger().info("RPi.GPIO imported successfully")
        except (ImportError, RuntimeError) as e:
            self.gpio_available = False
            self.get_logger().error(f"Failed to import RPi.GPIO: {str(e)}")
            self.get_logger().warning("Running in simulation mode")

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_frequency', 500),
                ('max_pwm', 15),
                ('min_pwm', 5),
                ('max_linear_vel', 0.5),
                ('max_angular_vel', 1.0),
                ('pwm_left_pin', 18),
                ('dir_left_pin', 23),
                ('pwm_right_pin', 13),
                ('dir_right_pin', 24),
                ('left_motor_inverted', True),
                ('right_motor_inverted', False),
                ('cmd_vel_timeout', 0.5),  # NEW parameter: timeout in seconds
            ])

        # Load parameters
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value

        # GPIO Pins
        self.PWM_LEFT = self.get_parameter('pwm_left_pin').value
        self.DIR_LEFT = self.get_parameter('dir_left_pin').value
        self.PWM_RIGHT = self.get_parameter('pwm_right_pin').value
        self.DIR_RIGHT = self.get_parameter('dir_right_pin').value

        self.left_inverted = self.get_parameter('left_motor_inverted').value
        self.right_inverted = self.get_parameter('right_motor_inverted').value

        # Setup GPIO
        if self.gpio_available:
            try:
                self._setup_gpio()
            except Exception as e:
                self.gpio_available = False
                self.get_logger().error(f"GPIO init failed: {str(e)}")
                self.get_logger().warning("Continuing in simulation mode")

        # ROS subscription and timeout checker
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self._check_cmd_timeout)  # check every 100ms

        self.get_logger().info(" Motor Driver Node initialized")

    def _setup_gpio(self):
        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)
        for pin in [self.PWM_LEFT, self.DIR_LEFT, self.PWM_RIGHT, self.DIR_RIGHT]:
            self.GPIO.setup(pin, self.GPIO.OUT)

        self.pwm_left = self.GPIO.PWM(self.PWM_LEFT, self.pwm_freq)
        self.pwm_right = self.GPIO.PWM(self.PWM_RIGHT, self.pwm_freq)
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        self.get_logger().info("GPIO and PWM initialized")

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular_z = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        left_speed = linear_x - (angular_z * 0.25)
        right_speed = linear_x + (angular_z * 0.25)

        left_pwm = self._velocity_to_pwm(left_speed)
        right_pwm = self._velocity_to_pwm(right_speed)

        if self.gpio_available:
            self._set_motor('left', left_pwm, left_speed >= 0)
            self._set_motor('right', right_pwm, right_speed >= 0)

    def _check_cmd_timeout(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > int(self.cmd_timeout * 1e9):
            if self.gpio_available:
                self.pwm_left.ChangeDutyCycle(0)
                self.pwm_right.ChangeDutyCycle(0)

    def _velocity_to_pwm(self, velocity):
        if abs(velocity) < 0.001:
            return 0.0
        pwm = (abs(velocity) / self.max_linear) * self.max_pwm
        return max(self.min_pwm, min(pwm, self.max_pwm))

    def _set_motor(self, motor, pwm_value, forward=True):
        try:
            if motor == 'left':
                pwm = self.pwm_left
                dir_pin = self.DIR_LEFT
                inverted = self.left_inverted
            else:
                pwm = self.pwm_right
                dir_pin = self.DIR_RIGHT
                inverted = self.right_inverted

            direction = self.GPIO.HIGH if (not forward) ^ inverted else self.GPIO.LOW
            self.GPIO.output(dir_pin, direction)
            pwm.ChangeDutyCycle(pwm_value if pwm_value > 0 else 0)

        except Exception as e:
            self.get_logger().error(f"Error on motor {motor}: {str(e)}")

    def destroy_node(self):
        if self.gpio_available:
            try:
                self.pwm_left.stop()
                self.pwm_right.stop()
                self.GPIO.cleanup()
                self.get_logger().info(" GPIO cleaned up")
            except Exception as e:
                self.get_logger().error(f" Cleanup error: {str(e)}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        motor_driver = MotorDriverNode()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        motor_driver.get_logger().info("Stopped by user")
    finally:
        if 'motor_driver' in locals():
            motor_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
