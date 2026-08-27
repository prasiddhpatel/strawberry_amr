cmake_minimum_required(VERSION 3.8)
project(safety_supervisor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
ament_target_dependencies(safety_supervisor_node
  rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS safety_supervisor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
