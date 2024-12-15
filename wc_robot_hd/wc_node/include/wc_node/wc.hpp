#ifndef WC_NODE__WC_HPP_
#define WC_NODE__WC_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace wc_ad
{
namespace wc
{
class Wc
{
public:
  explicit Wc(
    std::shared_ptr<rclcpp::Node> & nh,
    const double wheel_radius,
    const double hall_resolution,
    const std::string lidar_topic);

  virtual ~Wc() {}

private:
  bool calculate_odometry(const rclcpp::Duration & duration);
  void update_hall_sensor(const double hall_sensor_count);
  void update_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);

  void hall_sensor_callback(const std_msgs::msg::Float64::SharedPtr hall_sensor_msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);

  void publish(const rclcpp::Time & now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hall_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  double wheel_radius_;
  double hall_resolution_;
  std::string lidar_topic_;
  
  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;
  bool publish_tf_;

  double hall_sensor_count_;
  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;
};
}  // namespace wc
}  // namespace wc_ad

#endif  // WC_NODE__WC_HPP_
