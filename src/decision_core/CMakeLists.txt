cmake_minimum_required(VERSION 3.8)
project(decision_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(decision_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(decision_node
  src/decision_node.cpp
  knowledge/ontology.cpp
)

ament_target_dependencies(decision_node
  rclcpp
  decision_interfaces
  ament_index_cpp
)

target_link_libraries(decision_node
  yaml-cpp
)

target_include_directories(decision_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/knowledge
)

install(DIRECTORY knowledge/
  DESTINATION share/${PROJECT_NAME}/knowledge
)

install(TARGETS decision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
