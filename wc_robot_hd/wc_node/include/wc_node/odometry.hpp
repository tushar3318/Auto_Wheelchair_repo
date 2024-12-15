#ifndef WC_NODE__ODOMETRY_HPP_
#define WC_NODE__ODOMETRY_HPP_

#include <array>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float32_multi_array.hpp> // For hall sensor data

namespace wc_ad
{
namespace wc
{
class Odometry
{
public:
  explicit Odometry(
    std::shared_ptr<rclcpp::Node> & nh,
    const double wheels_separation,
    const double wheels_radius);
  virtual ~Odometry() {}

private:
  bool calculate_odometry(const rclcpp::Duration & duration);

  void hall_sensor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr hall_msg);
  void publish(const rclcpp::Time & now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hall_sensor_sub_;

  double wheels_separation_;
  double wheels_radius_;

  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;

  bool publish_tf_;

  std::array<double, 2> diff_wheel_positions_;
  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;
};
} // namespace wc
} // namespace wc_ad

#endif // WC_NODE__ODOMETRY_HPP_
