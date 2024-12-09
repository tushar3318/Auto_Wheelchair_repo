#ifndef WC__DIFF_DRIVE_CONTROLLER_HPP_
#define WC_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "wc_node/odometry.hpp"

namespace wc_ad
{
namespace wc
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  
}  
#endif  