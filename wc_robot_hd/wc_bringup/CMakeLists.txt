cmake_minimum_required(VERSION 3.8)
project(wc_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/motor_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
