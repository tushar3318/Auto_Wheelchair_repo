#include "wc_node/odometry.hpp"

using namespace std::chrono_literals;

namespace wc_ad
{
namespace wc
{

Odometry::Odometry(
  std::shared_ptr<rclcpp::Node> & nh,
  const double wheels_separation,
  const double wheels_radius)
: nh_(nh),
  wheels_separation_(wheels_separation),
  wheels_radius_(wheels_radius),
  publish_tf_(true)
{
  RCLCPP_INFO(nh_->get_logger(), "Initializing Odometry");

  nh_->declare_parameter<std::string>("odometry.frame_id", "odom");
  nh_->declare_parameter<std::string>("odometry.child_frame_id", "base_link");
  nh_->declare_parameter<bool>("odometry.publish_tf", true);

  nh_->get_parameter("odometry.frame_id", frame_id_of_odometry_);
  nh_->get_parameter("odometry.child_frame_id", child_frame_id_of_odometry_);
  nh_->get_parameter("odometry.publish_tf", publish_tf_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  hall_sensor_sub_ = nh_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "hall_sensor_data", qos,
    std::bind(&Odometry::hall_sensor_callback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
}

void Odometry::hall_sensor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr hall_msg)
{
  static rclcpp::Time last_time = nh_->now();
  rclcpp::Time current_time = nh_->now();
  rclcpp::Duration duration = current_time - last_time;

  // Assume hall_msg->data[0] and hall_msg->data[1] are left and right wheel positions
  diff_wheel_positions_[0] = hall_msg->data[0];
  diff_wheel_positions_[1] = hall_msg->data[1];

  if (calculate_odometry(duration)) {
    publish(current_time);
  }

  last_time = current_time;
}

bool Odometry::calculate_odometry(const rclcpp::Duration & duration)
{
  double wheel_l = diff_wheel_positions_[0];
  double wheel_r = diff_wheel_positions_[1];

  double delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;
  double delta_theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;

  double step_time = duration.seconds();
  if (step_time == 0.0) return false;

  // Update robot pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + delta_theta / 2.0);
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + delta_theta / 2.0);
  robot_pose_[2] += delta_theta;

  // Update velocities
  robot_vel_[0] = delta_s / step_time;
  robot_vel_[2] = delta_theta / step_time;

  return true;
}

void Odometry::publish(const rclcpp::Time & now)
{
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = frame_id_of_odometry_;
  odom_msg.child_frame_id = child_frame_id_of_odometry_;

  odom_msg.pose.pose.position.x = robot_pose_[0];
  odom_msg.pose.pose.position.y = robot_pose_[1];
  odom_msg.pose.pose.orientation.z = sin(robot_pose_[2] / 2.0);
  odom_msg.pose.pose.orientation.w = cos(robot_pose_[2] / 2.0);

  odom_msg.twist.twist.linear.x = robot_vel_[0];
  odom_msg.twist.twist.angular.z = robot_vel_[2];

  odom_pub_->publish(odom_msg);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = now;
    odom_tf.header.frame_id = frame_id_of_odometry_;
    odom_tf.child_frame_id = child_frame_id_of_odometry_;

    odom_tf.transform.translation.x = robot_pose_[0];
    odom_tf.transform.translation.y = robot_pose_[1];
    odom_tf.transform.rotation.z = sin(robot_pose_[2] / 2.0);
    odom_tf.transform.rotation.w = cos(robot_pose_[2] / 2.0);

    tf_broadcaster_->sendTransform(odom_tf);
  }
}

} // namespace wc
} // namespace wc_ad
