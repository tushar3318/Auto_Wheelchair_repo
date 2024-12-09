#ifndef WC__DEVICES__SOUND_HPP_
#define WC__DEVICES__SOUND_HPP_

#include <wc_msgs/srv/sound.hpp>
#include <memory>
#include <string>
#include "wc_node/devices/devices.hpp"

namespace wc_ad
{
namespace wc
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<wc_msgs::srv::Sound>::SharedPtr client,
    wc_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<wc_msgs::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace wc
}  
#endif 