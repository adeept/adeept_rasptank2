cmake_minimum_required(VERSION 3.8)
project(rasptank_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
