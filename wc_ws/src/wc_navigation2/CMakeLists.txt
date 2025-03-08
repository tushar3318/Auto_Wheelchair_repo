cmake_minimum_required(VERSION 3.8)
project(wc_navigation2)

cmake_minimum_required(VERSION 3.8)
project(wc_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
