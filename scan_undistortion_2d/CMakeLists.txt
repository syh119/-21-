cmake_minimum_required(VERSION 3.8)
project(scan_undistortion_2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_undistortion_node
  src/scan_undistortion_node.cpp
)

ament_target_dependencies(scan_undistortion_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  scan_undistortion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
