#include <chrono>
#include <memory>
#include <string>

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include "wc_node/diff_drive_controller.hpp"
#include "wc_node/wc.hpp"

void print_help()
{
  printf("For WC Node:\n");
  printf("Usage: wc_node [-i usb_port] [-h]\n");
  printf("Options:\n");
  printf("  -h : Print this help message.\n");
  printf("  -i usb_port : Specify the USB port connected to OpenCR (default: /dev/ttyACM0).\n");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Handle help option
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  // Parse USB port option
  std::string usb_port = "/dev/ttyACM0";  // Default USB port
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (cli_option != nullptr) {
    usb_port = std::string(cli_option);
  }

  // Create the ROS 2 executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Initialize WC Node and DiffDriveController
  auto node = std::make_shared<rclcpp::Node>("wc_node");
  double wheel_radius = 0.1;    // Example wheel radius in meters
  double hall_resolution = 100; // Example hall sensor resolution
  std::string lidar_topic = "/scan"; // Example LiDAR topic

  auto wc = std::make_shared<wc_ad::wc::Wc>(
    node, wheel_radius, hall_resolution, lidar_topic);

  auto diff_drive_controller = std::make_shared<wc_ad::wc::DiffDriveController>(
    wc->get_wheels()->separation,
    wc->get_wheels()->radius);

  // Add nodes to the executor
  executor.add_node(node);
  executor.add_node(wc);
  executor.add_node(diff_drive_controller);

  // Start spinning
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
