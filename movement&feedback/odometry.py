import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("The odometry node has started.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return

        self.odom_vel_pub = self.create_publisher(Twist, "/odometrydata", 10)
        self.timer = self.create_timer(0.1, self.send_odometry_data)

    def send_odometry_data(self):
        try:
            data = self.serial_port.readline().decode('utf-8').strip()
            if not data:
                return

            parts = data.split('|')
            if len(parts) != 2:
                self.get_logger().warn(f"Malformed data received: {data}")
                return

            R,L=10.16,54
            left_vel = (2 * 3.14159265 *int(parts[0])*R)/60
            right_vel = (2 * 3.14159265 *int(parts[1])*R)/60
            linear_velocity = (left_vel + right_vel) / 2
            angular_velocity = (right_vel - left_vel) /L


            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity

            self.odom_vel_pub.publish(msg)
            self.get_logger().info(f"Published: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}")

        except (ValueError, serial.SerialException) as e:
            self.get_logger().error(f"Error parsing data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    odom_node = Odometry()

    if not hasattr(odom_node, "serial_port"):  # If serial failed, don't run
        return

    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        odom_node.get_logger().info("Shutting down odometry node.")
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
