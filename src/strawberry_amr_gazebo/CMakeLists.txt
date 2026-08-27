cmake_minimum_required(VERSION 3.8)
project(strawberry_amr_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds scripts rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
