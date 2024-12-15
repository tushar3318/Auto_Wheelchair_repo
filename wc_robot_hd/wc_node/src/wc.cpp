#include "wc_node/wc.hpp"

using wc_ad::wc::Wc;
using namespace std::chrono_literals;

Wc::Wc(
  std::shared_ptr<rclcpp::Node> & nh,
  const double wheel_radius,
  const double hall_resolution,
  const std::string lidar_topic)
: nh_(nh),
  wheel_radius_(wheel_radius),
  hall_resolution_(hall_resolution),
  lidar_topic_(lidar_topic),
  publish_tf_(false),
  hall_sensor_count_(0.0)
{
  RCLCPP_INFO(nh_->get_logger(), "Initializing WC Node");

  nh_->declare_parameter<std::string>("odometry.frame_id", "odom");
  nh_->declare_parameter<std::string>("odometry.child_frame_id", "base_link");
  nh_->declare_parameter<bool>("odometry.publish_tf", false);

  nh_->get_parameter("odometry.frame_id", frame_id_of_odometry_);
  nh_->get_parameter("odometry.child_frame_id", child_frame_id_of_odometry_);
  nh_->get_parameter("odometry.publish_tf", publish_tf_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

  hall_sensor_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
    "hall_sensor",
    qos,
    std::bind(&Wc::hall_sensor_callback, this, std::placeholders::_1));

  lidar_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic_,
    qos,
    std::bind(&Wc::lidar_callback, this, std::placeholders::_1));
}

void Wc::hall_sensor_callback(const std_msgs::msg::Float64::SharedPtr hall_sensor_msg)
{
  static rclcpp::Time last_time = nh_->now();
  const rclcpp::Time current_time = nh_->now();
  const rclcpp::Duration duration = current_time - last_time;

  hall_sensor_count_ = hall_sensor_msg->data;

  calculate_odometry(duration);
  publish(current_time);

  last_time = current_time;
}

void Wc::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  update_lidar_data(lidar_msg);
}

bool Wc::calculate_odometry(const rclcpp::Duration & duration)
{
  double delta_s = (hall_sensor_count_ / hall_resolution_) * 2 * M_PI * wheel_radius_;
  static double last_theta = 0.0;
  double delta_theta = 0.0;

  double step_time = duration.seconds();
  if (step_time == 0.0) {
    return false;
  }

  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  robot_vel_[0] = delta_s / step_time;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = delta_theta / step_time;

  return true;
}

void Wc::publish(const rclcpp::Time & now)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);
  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;
  odom_tf.transform.translation.x = robot_pose_[0];
  odom_tf.transform.translation.y = robot_pose_[1];
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_pub_->publish(std::move(odom_msg));

  if (publish_tf_) {
    tf_broadcaster_->sendTransform(odom_tf);
  }
}

void Wc::update_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  RCLCPP_DEBUG(nh_->get_logger(), "LiDAR data received. Range count: %zu", lidar_msg->ranges.size());
}

