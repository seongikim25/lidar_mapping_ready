cmake_minimum_required(VERSION 3.8)
project(ax12_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(ax12_joint_state_node
  src/ax12_joint_state_node.cpp
)

ament_target_dependencies(
  ax12_joint_state_node
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

install(TARGETS
  ax12_joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

