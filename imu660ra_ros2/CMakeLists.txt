cmake_minimum_required(VERSION 3.8)
project(imu660ra_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu660ra_node
  src/imu660ra_driver.cpp
  src/imu660ra_node.cpp
)

target_compile_features(imu660ra_node PUBLIC cxx_std_17)
target_include_directories(imu660ra_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu660ra_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)

install(
  TARGETS imu660ra_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(
  FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
