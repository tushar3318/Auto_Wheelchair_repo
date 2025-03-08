import rclpy
import serial
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations as tf

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('wc_odometry')

        # Open serial connection with Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return

        # Parameters
        self.declare_parameter('wheel_radius', 0.15)  # 15 cm wheels
        self.declare_parameter('wheel_base', 0.5)     # Distance between wheels

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to update odometry at 10Hz
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10Hz

    def get_wheel_rpm(self):
        """Reads RPM values from Arduino over Serial."""
        try:
            data = self.serial_port.readline().decode('utf-8').strip()
            if not data:
                return None, None

            parts = data.split('|')
            if len(parts) != 2:
                self.get_logger().warn(f"Malformed data received: {data}")
                return None, None

            rpm_left = int(parts[0])
            rpm_right = int(parts[1])

            return rpm_left, rpm_right

        except (ValueError, serial.SerialException) as e:
            self.get_logger().error(f"Error reading serial data: {str(e)}")
            return None, None

    def update_odometry(self):
        rpm_left, rpm_right = self.get_wheel_rpm()
        if rpm_left is None or rpm_right is None:
            return  # Skip if no valid data

        dt = 0.1  # Timer interval in seconds

        # Convert RPM to wheel velocity (m/s)
        v_left = (rpm_left * 2 * math.pi * self.wheel_radius) / 60.0
        v_right = (rpm_right * 2 * math.pi * self.wheel_radius) / 60.0

        # Compute robot velocity
        linear_velocity = (v_left + v_right) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_base

        # Integrate position using Euler method
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        q = tf.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        self.get_logger().info(f"Published: linear_x={linear_velocity:.2f}, angular_z={angular_velocity:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    if not hasattr(node, "serial_port"):  # If serial failed, don't run
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down odometry node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()