#ifndef WC_HPP
#define WC_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace wc_ad {
namespace wc {

class Wc : public rclcpp::Node {
public:
  // Struct to hold wheel properties
  struct Wheels {
    double separation;
    double radius;
  };

private:
  std::shared_ptr<rclcpp::Node> nh_;
  double wheel_radius_;
  double hall_resolution_;
  std::string lidar_topic_;
  bool publish_tf_;
  double hall_sensor_count_;
  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;
  
  // Wheel properties
  Wheels wheels_;
  
  // Odometry variables
  double robot_pose_[3] = {0.0, 0.0, 0.0};  // [x, y, theta]
  double robot_vel_[3] = {0.0, 0.0, 0.0};   // [linear_x, linear_y, angular_z]

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hall_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

public:
  Wc(std::shared_ptr<rclcpp::Node> & nh, double wheel_radius, double hall_resolution, const std::string lidar_topic);

  // Accessor for wheel properties
  Wheels* get_wheels() { return &wheels_; }

private:
  void hall_sensor_callback(const std_msgs::msg::Float64::SharedPtr hall_sensor_msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);
  bool calculate_odometry(const rclcpp::Duration & duration);
  void publish(const rclcpp::Time & now);
  void update_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);
};

}  // namespace wc
}  // namespace wc_ad

#endif  // WC_HPP
