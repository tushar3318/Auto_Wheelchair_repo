cmake_minimum_required(VERSION 3.8)
project(wc_node)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wc_msgs REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  #"src/devices/motor_power.cpp"
  #"src/devices/sound.cpp"
  #"src/devices/reset.cpp"

  "src/diff_drive_controller.cpp"
  "src/odometry.cpp"
  "src/wc.cpp"

  #"src/sensors/battery_state.cpp"
  #"src/sensors/imu.cpp"
  #"src/sensors/joint_state.cpp"
  #"src/sensors/sensor_state.cpp"
)

set(DEPENDENCIES
  "dynamixel_sdk"
  "geometry_msgs"
  "message_filters"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "wc_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "wc_robot_wc")

add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})


install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_package()
