cmake_minimum_required(VERSION 3.8)
project(wc_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

set(msg_files
  "msg/SensorState.msg"
  "msg/Sound.msg"
)

set(srv_files
  "srv/Sound.srv"
  "srv/Dqn.srv"
)

set(action_files
  "action/Patrol.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
