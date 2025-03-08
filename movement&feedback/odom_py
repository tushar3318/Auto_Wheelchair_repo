import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations as tf
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('wc_odometry')

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

        # Timer callback
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10Hz

    def get_wheel_rpm(self):
        """Dummy function to simulate getting RPM from Hall sensors."""
        rpm_left = 30.0  # Example value
        rpm_right = 32.0 # Example value
        return rpm_left, rpm_right

    def update_odometry(self):
        rpm_left, rpm_right = self.get_wheel_rpm()
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

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
