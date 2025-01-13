#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial # For communication with Arduino

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Initialize serial communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyACM2', 9600, timeout=1)  # Adjust port and baud rate
        
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward speed
       # linear_y = msg.linear.y  # Forward speed
        angular_z = msg.angular.z  # Rotational speed
        
        # Calculate left and right motor speeds
        left_speed = max(0, linear_x - (0.265*angular_z))  # Prevent negative speeds
        right_speed = max(0, linear_x + (0.265*angular_z))

        # motor specific.... change accordingly
        # Map speeds to PWM range (46 - 82)
        left_pwm = int(self.speed_to_pwm(left_speed, 46, 85))  # Map to the desired range
        right_pwm = int(self.speed_to_pwm(right_speed, 46, 85))
        
        # Send PWM values to Arduino as LXXXRXXX (e.g., L127R130)
        command = f"L{left_pwm:03d}R{right_pwm:03d}\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent command to Arduino: {command.strip()}")
    
    def speed_to_pwm(self, speed, min_pwm, max_pwm):
        # Map speed (0.0 to 1.0) to PWM range (min_pwm to max_pwm)
        return min_pwm + ((speed/(0.75)) * (max_pwm - min_pwm))
    
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
